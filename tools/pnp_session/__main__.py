"""`cd tools && uv run python -m pnp_session lap-c0 ...`: the same commands as
tools/pnp_session_report.py."""

from __future__ import annotations

from pnp_session.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
