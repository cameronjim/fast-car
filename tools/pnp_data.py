#!/usr/bin/env python3
"""pnp_data: the runs of a bag, the split of a new session by the registered rule, the guard
training code calls, and a wrapper for the dynamics confirmation.

    uv run --project tools --extra session python tools/pnp_data.py runs <bag>
    uv run --project tools --extra session python tools/pnp_data.py split <bag...> --session <date>
    uv run --project tools --extra session python tools/pnp_data.py guard <bag name...>
    uv run --project tools --extra session python tools/pnp_data.py confirm --bag <bag> --candidate <bundle> --deployed <bundle>

Pre-registration PnP v0.4 sections 0.3 and 1 (brief 36). Reads bags and writes one split file; it
changes nothing on the car.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pnp_data.cli import main

if __name__ == "__main__":
    sys.exit(main())
