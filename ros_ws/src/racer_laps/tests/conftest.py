"""hypothesis is a dev dependency (uv run pytest). `colcon test` runs this directory too, under
the ros-dev image's system python3 where it is absent: the property test is not collected there
instead of failing the whole directory's collection."""

from __future__ import annotations

import importlib.util

collect_ignore = [] if importlib.util.find_spec("hypothesis") else ["test_cap_property.py"]
