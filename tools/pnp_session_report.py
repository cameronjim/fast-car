#!/usr/bin/env python3
"""Timing report and lap check for the car session (brief 34, pre-registration PnP v0.3).

    uv run --project tools --extra session python tools/pnp_session_report.py timing --bag <dir> [--out report.json]
    uv run --project tools --extra session python tools/pnp_session_report.py laps   --bag <dir>   # D3-C0
    uv run --project tools --extra session python tools/pnp_session_report.py host-facts --out-dir <dir>

`timing` prints the tables of section 5 (M-T) for every PnP run and pooled: `/drive_raw` gaps by
receive time and header stamp, planner solve time and tick gaps, `/safety/events` by source, the
watchdog what-if, the rate limiter what-if (needs limiter_whatif_cli, built from
ros_ws/src/racer_session_tools) and the v0.2 4.2 acceptance. It reads a bag; nothing here talks to a
car. Exit status 3 when the 2026-11-13 baseline bag does not reproduce the pre-registered counts.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    from pnp_session.cli import main  # noqa: E402
except ModuleNotFoundError as exc:  # racer_eval and scipy come with the `session` extra only
    if exc.name and exc.name.split(".")[0] in ("racer_eval", "scipy"):
        sys.exit(
            "pnp_session_report needs the `session` extra (racer_eval, scipy): run "
            "`uv run --project tools --extra session python tools/pnp_session_report.py ...`"
        )
    raise

if __name__ == "__main__":
    sys.exit(main())
