#!/usr/bin/env python3
"""lapref: the laps of one whole run from the gated steering command and wheel distance only.

    uv run --project tools --extra session python tools/lapref.py <bag> --run <start> <end>
    uv run --project tools --extra session python tools/lapref.py <bag> --split-run <name>

Offline, not causal, cannot run on the car (pre-registration PnP v0.4 section 4.2, brief 36). The
method is tools/lapref/method.py; docs/notes/lap-reference-2026-11.md describes it.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from lapref.cli import main

if __name__ == "__main__":
    sys.exit(main())
