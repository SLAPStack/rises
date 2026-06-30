"""Shared fixtures for the orchestration test suites.

The ``orchestration`` package uses relative imports, so tests must
import it as ``orchestration.<module>`` from the repository root rather
than via files in this directory. This conftest prepends the repo root
to :data:`sys.path` so that resolution succeeds in any working
directory.
"""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
