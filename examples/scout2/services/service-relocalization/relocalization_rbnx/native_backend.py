"""Check native module location and runtime compatibility before registration."""
from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path
import sys


def verify(directory: Path):
    """Load this package's native build using the actual service interpreter.

    Reject a missing build or an extension found elsewhere on PYTHONPATH. An
    actual import exposes incompatible ABI and missing shared libraries before
    Atlas registration. This proves deployment availability only, not sensor
    readiness, map consistency or localization validity. Import errors propagate.
    """
    expected = Path(directory).resolve(strict=True)
    spec = importlib.util.find_spec('_relocalization_bbs')
    if spec is None or spec.origin is None:
        raise ImportError('native BBS/ICP module is missing; run scripts/build.sh')
    origin = Path(spec.origin).resolve(strict=True)
    if origin.parent != expected:
        raise ImportError(f'native BBS/ICP module is outside the package build: {origin}')
    module = importlib.import_module('_relocalization_bbs')
    for name in ('BbsSearch', 'PlanarRegistration', 'SearchOptions',
                 'RegistrationOptions', 'CancellationToken', 'SearchCancelled',
                 'SearchTimeout', 'RegistrationCancelled', 'RegistrationTimeout',
                 'RegistrationRejected'):
        if not hasattr(module, name):
            raise ImportError(f'native BBS/ICP module lacks {name}; rebuild the package')
    module.SearchOptions()
    module.RegistrationOptions()
    token = module.CancellationToken()
    if token.cancelled:
        raise RuntimeError('native cancellation token starts cancelled')
    token.cancel()
    if not token.cancelled:
        raise RuntimeError('native cancellation token failed to cancel')
    return module


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit('expected the package native build directory')
    try:
        verify(Path(sys.argv[1]))
    except (ImportError, OSError, RuntimeError) as exc:
        raise SystemExit(f'[relocalization] native BBS/ICP startup check failed: {exc}')
