"""Durable one-time face-message tasks, separate from the registered face mapping."""
from __future__ import annotations

import sqlite3
import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator


class MessageStore:
    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._connect() as connection:
            connection.execute("""
                CREATE TABLE IF NOT EXISTS messages (
                    task_id TEXT PRIMARY KEY,
                    person_id INTEGER NOT NULL,
                    person_name TEXT NOT NULL,
                    sender TEXT NOT NULL,
                    body TEXT NOT NULL,
                    object_id TEXT NOT NULL DEFAULT '',
                    object_label TEXT NOT NULL DEFAULT '',
                    object_status TEXT NOT NULL DEFAULT 'not_tracked',
                    object_last_seen_at REAL,
                    object_checked_at REAL,
                    status TEXT NOT NULL,
                    created_at REAL NOT NULL,
                    event_id TEXT NOT NULL DEFAULT '',
                    delivered_at REAL,
                    detail TEXT NOT NULL DEFAULT ''
                )
            """)
            connection.execute("CREATE INDEX IF NOT EXISTS messages_pending ON messages(person_id, status)")
            connection.execute(
                "UPDATE messages SET status='uncertain', detail='skill restarted during delivery; verify before retrying' "
                "WHERE status='delivering'"
            )
            columns = {row[1] for row in connection.execute("PRAGMA table_info(messages)")}
            migrations = {
                "object_id": "ALTER TABLE messages ADD COLUMN object_id TEXT NOT NULL DEFAULT ''",
                "object_label": "ALTER TABLE messages ADD COLUMN object_label TEXT NOT NULL DEFAULT ''",
                "object_status": "ALTER TABLE messages ADD COLUMN object_status TEXT NOT NULL DEFAULT 'not_tracked'",
                "object_last_seen_at": "ALTER TABLE messages ADD COLUMN object_last_seen_at REAL",
                "object_checked_at": "ALTER TABLE messages ADD COLUMN object_checked_at REAL",
            }
            for column, statement in migrations.items():
                if column not in columns:
                    connection.execute(statement)

    @contextmanager
    def _connect(self) -> Iterator[sqlite3.Connection]:
        connection = sqlite3.connect(self.path, timeout=5)
        connection.row_factory = sqlite3.Row
        try:
            with connection:
                yield connection
        finally:
            connection.close()

    @staticmethod
    def _rows(cursor: sqlite3.Cursor) -> list[dict]:
        return [dict(row) for row in cursor.fetchall()]

    @staticmethod
    def _dedup_text(value: str) -> str:
        return "".join(value.split()).rstrip("。！？!?；;，,.")

    def create_or_get(
        self,
        person_id: int,
        person_name: str,
        sender: str,
        body: str,
        object_id: str = "",
        object_label: str = "",
    ) -> tuple[str, bool]:
        """Atomically deduplicate an identical pending task before saving a new one."""
        task_id = str(uuid.uuid4())
        with self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            existing_rows = connection.execute(
                "SELECT task_id, sender, body, object_id FROM messages WHERE person_id=? AND status='pending' "
                "ORDER BY created_at, task_id", (person_id,),
            ).fetchall()
            sender_key = self._dedup_text(sender)
            body_key = self._dedup_text(body)
            for row in existing_rows:
                if (
                    self._dedup_text(row["sender"]) == sender_key
                    and self._dedup_text(row["body"]) == body_key
                    and str(row["object_id"] or "") == object_id
                ):
                    return str(row["task_id"]), False
            existing = connection.execute(
                "SELECT COALESCE(SUM(LENGTH(body) + LENGTH(sender)), 0) FROM messages "
                "WHERE person_id=? AND status='pending'", (person_id,),
            ).fetchone()[0]
            if existing + len(body) + len(sender) > 180:
                raise ValueError("pending messages for this person exceed 180 characters; deliver or cancel them first")
            connection.execute(
                "INSERT INTO messages(task_id, person_id, person_name, sender, body, object_id, object_label, "
                "object_status, status, created_at) VALUES(?, ?, ?, ?, ?, ?, ?, ?, 'pending', ?)",
                (
                    task_id, person_id, person_name, sender, body, object_id, object_label,
                    "present" if object_id else "not_tracked", time.time(),
                ),
            )
        return task_id, True

    def create(self, person_id: int, person_name: str, sender: str, body: str) -> str:
        """Save a pending task and return an existing identical pending task when present."""
        task_id, _ = self.create_or_get(person_id, person_name, sender, body)
        return task_id

    def update_object_status(
        self,
        task_id: str,
        object_status: str,
        checked_at: float,
        last_seen_at: float | None,
    ) -> None:
        """Persist the latest Scene existence result for a tracked object."""
        with self._connect() as connection:
            connection.execute(
                "UPDATE messages SET object_status=?, object_checked_at=?, object_last_seen_at=? "
                "WHERE task_id=? AND object_id != ''",
                (object_status, checked_at, last_seen_at, task_id),
            )

    def claim(self, person_id: int, observed_at: float, event_id: str) -> list[dict]:
        """Only claim tasks created before this event entered the service's queue."""
        with self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            rows = self._rows(connection.execute(
                "SELECT * FROM messages WHERE person_id=? AND status='pending' AND created_at<=? "
                "ORDER BY created_at, task_id", (person_id, observed_at),
            ))
            for row in rows:
                connection.execute(
                    "UPDATE messages SET status='delivering', event_id=? WHERE task_id=? AND status='pending'",
                    (event_id, row["task_id"]),
                )
        return rows

    def finish(self, task_ids: list[str], speech_ok: bool, detail: str) -> None:
        """Mark only confirmed speech as delivered; unknown outcomes never auto-retry."""
        status = "delivered" if speech_ok else "uncertain"
        with self._connect() as connection:
            connection.executemany(
                "UPDATE messages SET status=?, delivered_at=?, detail=? WHERE task_id=? AND status='delivering'",
                [(status, time.time() if speech_ok else None, detail[:500], task_id) for task_id in task_ids],
            )

    def get(self, task_id: str) -> dict | None:
        with self._connect() as connection:
            row = connection.execute("SELECT * FROM messages WHERE task_id=?", (task_id,)).fetchone()
            return dict(row) if row else None

    def list(self, status: str = "") -> list[dict]:
        with self._connect() as connection:
            if status:
                cursor = connection.execute(
                    "SELECT * FROM messages WHERE status=? ORDER BY created_at DESC LIMIT 100", (status,)
                )
            else:
                cursor = connection.execute("SELECT * FROM messages ORDER BY created_at DESC LIMIT 100")
            return self._rows(cursor)

    def cancel(self, task_id: str) -> bool:
        with self._connect() as connection:
            cursor = connection.execute(
                "UPDATE messages SET status='cancelled', detail='cancelled by user' "
                "WHERE task_id=? AND status='pending'", (task_id,),
            )
            return cursor.rowcount == 1
