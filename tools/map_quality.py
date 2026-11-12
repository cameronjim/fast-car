"""Map quality numbers for saved slam_toolbox maps (map_server yaml + pgm).

    uv run --project tools python tools/map_quality.py MAP.yaml [MAP.yaml ...] [--json]

Prints, per map, the numbers of mapping_eval/quality.py: wall cells, wall thickness (p50, p90),
interior wall cells (a wall with free floor on both sides: the second copy of a doubled map),
free regions. See docs/notes/mapping-2026-11-12.md for what healthy and broken maps read.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from mapping_eval.quality import load_map_strict, map_metrics


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("maps", nargs="+", type=Path)
    parser.add_argument("--json", action="store_true", help="one JSON object per line")
    args = parser.parse_args(argv)
    for path in args.maps:
        metrics = map_metrics(load_map_strict(path))
        if args.json:
            print(json.dumps({"map": str(path), **metrics}))
        else:
            print(
                f"{path.name}: cells {metrics['wall_cells']}, thickness p50 "
                f"{metrics['thickness_p50_m'] * 100:.0f} cm p90 {metrics['thickness_p90_m'] * 100:.0f} cm, "
                f"layers>=2 {metrics['multi_layer_fraction'] * 100:.1f} % (mean {metrics['layers_mean']:.2f}), "
                f"interior {metrics['interior_wall_cells']} "
                f"({metrics['interior_fraction'] * 100:.1f} %), free regions {metrics['free_regions']}"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
