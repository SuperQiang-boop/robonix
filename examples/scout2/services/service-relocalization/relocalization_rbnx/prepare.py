"""Offline preparation of saved snapshots; never open a mapping runtime database."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sqlite3
import subprocess
import tempfile
import uuid

from .map_index import ARTIFACTS, build_index, canonical, sha256, verify_index

PKG = Path(__file__).resolve().parents[1]


def saved_snapshot(maps_dir: Path, map_id: str) -> tuple[Path, int]:
    """Restrict input to a named saved map with an explicit coordinate generation."""
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}", map_id):
        raise ValueError("invalid map_id")
    root = maps_dir.resolve(strict=True)
    database = (root / map_id / "rtabmap.db").resolve(strict=True)
    if not database.is_relative_to(root) or ".runtime" in database.parts:
        raise ValueError("only saved databases inside maps_dir may be prepared")
    if (root / map_id).is_symlink() or (root / map_id / "rtabmap.db").is_symlink():
        raise ValueError("saved map directory and database must not be symbolic links")
    if database.parent != (root / map_id).resolve():
        raise ValueError("database must belong to the named saved map")
    for suffix in ("-wal", "-shm", "-journal"):
        if database.with_name(database.name + suffix).exists():
            raise ValueError("snapshot has SQLite sidecars; save a stable checkpoint first")
    generation = int((database.parent / "generation").read_text().strip())
    if not 0 <= generation < 2**64:
        raise ValueError("map generation must be a uint64")
    return database, generation


def snapshot_stat(path: Path) -> tuple:
    value = path.stat()
    return value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns


def check_copy(database: Path) -> None:
    """Inspect SQLite metadata on the private copy; never decode sensor BLOBs here."""
    connection = sqlite3.connect(database.as_uri() + "?mode=ro&immutable=1", uri=True)
    try:
        if connection.execute("PRAGMA integrity_check").fetchall() != [("ok",)]:
            raise ValueError("snapshot SQLite integrity check failed")
        if connection.execute("SELECT version FROM Admin").fetchall() != [("0.23.7",)]:
            raise ValueError("only RTAB-Map database 0.23.7 is validated")
    finally:
        connection.close()


def export_snapshot(database: Path, output: Path, exporter: Path, image: str | None,
                    anchor_node: int, timeout_s: float) -> None:
    """Run the read-only C++ exporter with bounded execution and owned cleanup."""
    name = "relocalization-prepare-" + uuid.uuid4().hex
    if image:
        relative = exporter.relative_to(PKG)
        command = ["docker", "run", "--rm", "--name", name, "--network", "none",
                   "--read-only", "--tmpfs", "/tmp", "--user", f"{os.getuid()}:{os.getgid()}",
                   "--env", "LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib",
                   "--volume", f"{PKG}:/package:ro", "--volume", f"{database.parent}:/job",
                   "--entrypoint", f"/package/{relative}", image,
                   "/job/snapshot.db", "/job/export", str(anchor_node)]
    else:
        command = [str(exporter), str(database), str(output), str(anchor_node)]
    try:
        subprocess.run(command, check=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        if image:
            # The Docker client timeout alone does not stop its container.
            subprocess.run(["docker", "rm", "--force", name], check=True, timeout=15)
        raise


def sync_directory(path: Path) -> None:
    """Make directory entry changes durable before reporting a published index."""
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def prepare(maps_dir: Path, map_id: str, index_dir: Path, exporter: Path,
            *, image: str | None = None, voxel_size: float = 0.05,
            levels: int = 8, anchor_node: int = 0, timeout_s: float = 120) -> Path:
    """Publish one checked index atomically; reuse only exact, verified identities.

    Sources remain read-only. A same-key flock serializes concurrent preparers.
    Existing corrupt indices raise; they are never silently replaced or reused.
    """
    if not math.isfinite(voxel_size) or not 0.01 <= voxel_size <= 1:
        raise ValueError("voxel_size must be in [0.01, 1.0] metres")
    if type(levels) is not int or not 1 <= levels <= 16 or type(anchor_node) is not int or anchor_node < 0:
        raise ValueError("invalid pyramid levels or anchor node")
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= 3600:
        raise ValueError("timeout_s must be in (0, 3600]")
    database, generation = saved_snapshot(maps_dir, map_id)
    index_dir = index_dir.resolve()
    if index_dir == maps_dir.resolve() or index_dir.is_relative_to(maps_dir.resolve()):
        raise ValueError("index_dir must be outside read-only maps_dir")
    exporter = exporter.resolve(strict=True)
    if not exporter.is_file() or not os.access(exporter, os.X_OK):
        raise ValueError("build the executable exporter first")
    if image:
        try:
            image = subprocess.check_output(["docker", "image", "inspect", image,
                "--format", "{{.Id}}"], text=True, stderr=subprocess.PIPE, timeout=15).strip()
        except subprocess.CalledProcessError as exc:
            raise ValueError(
                f"snapshot exporter image {image} is unavailable: {exc.stderr.strip()}; "
                "run rbnx build -f robonix_manifest.yaml --no-update-check "
                "from the Scout2 deployment directory to rebuild the exporter") from exc
        if not re.fullmatch(r"sha256:[0-9a-f]{64}", image):
            raise ValueError("cannot resolve an immutable local image ID")
    initial_stat = snapshot_stat(database)
    digest = sha256(database)
    if snapshot_stat(database) != initial_stat:
        raise ValueError("saved snapshot changed while hashing")
    identity = dict(map_id=map_id, generation=generation, database_sha256=digest,
                    algorithm_version="rtabmap-export-index-v1", exporter_sha256=sha256(exporter),
                    image_id=image, voxel_size=voxel_size, pyramid_levels=levels, anchor_node=anchor_node)
    key = hashlib.sha256(canonical(identity)).hexdigest()
    parent = index_dir / map_id
    parent.mkdir(parents=True, exist_ok=True)
    destination = parent / key
    with (parent / f".{key}.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if destination.exists():
            verify_index(destination, identity)
            current, current_generation = saved_snapshot(maps_dir, map_id)
            if current != database or current_generation != generation or snapshot_stat(current) != initial_stat:
                raise ValueError("saved map changed during cached index verification")
            return destination
        with tempfile.TemporaryDirectory(prefix=".prepare-", dir=parent) as temporary:
            staging = Path(temporary)
            copy = staging / "snapshot.db"
            shutil.copyfile(database, copy)
            copy.chmod(0o444)
            if snapshot_stat(database) != initial_stat or sha256(copy) != digest:
                raise ValueError("snapshot changed while copying")
            check_copy(copy)
            output = staging / "export"
            try:
                export_snapshot(copy, output, exporter, image, anchor_node, timeout_s)
            finally:
                # Preserve useful graph diagnostics even when export refuses an
                # ambiguous map. Reports cannot be mistaken for ready indices.
                coverage = output / "coverage.json"
                if coverage.is_file():
                    reports = parent / "reports"
                    reports.mkdir(exist_ok=True)
                    diagnostic = staging / "coverage.json"
                    diagnostic.write_bytes(canonical(dict(identity=identity,
                        coverage=json.loads(coverage.read_text()))) + b"\n")
                    os.replace(diagnostic, reports / f"{key}.json")
            if sha256(copy) != digest:
                raise ValueError("exporter modified the snapshot copy")
            summary = build_index(output, voxel_size, levels)
            manifest = dict(format_version=1, identity=identity, summary=summary,
                files={name: sha256(output / name) for name in ARTIFACTS})
            (output / "index.json").write_bytes(canonical(manifest) + b"\n")
            verify_index(output, identity)
            current, current_generation = saved_snapshot(maps_dir, map_id)
            if current != database or current_generation != generation or snapshot_stat(current) != initial_stat:
                raise ValueError("saved map changed during index preparation")
            for path in output.iterdir():
                with path.open("rb") as stream:
                    os.fsync(stream.fileno())
            sync_directory(output)
            os.rename(output, destination)
            sync_directory(parent)
    return destination


def main() -> None:
    """Expose offline preparation without adding RPCs or changing robot deployment."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maps-dir", type=Path, required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--index-dir", type=Path, default=PKG / "data")
    parser.add_argument("--exporter", type=Path)
    parser.add_argument("--image", help="local build image, resolved to immutable ID; no pull")
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--levels", type=int, default=8)
    parser.add_argument("--anchor-node", type=int, default=0,
                        help="verified node in current map coordinates; 0 requires one component")
    parser.add_argument("--timeout-s", type=float, default=120)
    args = parser.parse_args()
    exporter = args.exporter or PKG / "rbnx-build" / (
        "backend-container" if args.image else "backend") / "relocalization_export"
    try:
        result = prepare(args.maps_dir, args.map_id, args.index_dir, exporter,
            image=args.image, voxel_size=args.voxel_size, levels=args.levels,
            anchor_node=args.anchor_node, timeout_s=args.timeout_s)
    except (OSError, ValueError, KeyError, sqlite3.Error, subprocess.SubprocessError) as exc:
        parser.exit(1, f"map preparation failed: {exc}\n")
    print(result)


if __name__ == "__main__":
    main()
