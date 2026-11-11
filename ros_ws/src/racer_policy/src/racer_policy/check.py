"""`python3 -m racer_policy.check <policy_dir>`: would policy_node start with this directory?

Runs `deploy.load_deployment` (contract checksum and version, the live vehicle_params
version, the observation schema, the sidecar settings) against the committed
`config/vehicle_params.yaml` through the generated binding, with no ROS and no torch. Exit 0
and `OK` on stdout if it passes; exit 1 and the specific refusal on stderr otherwise. The
racer app uses this, so the app only offers the ML policy button for a directory the node
itself would accept (the node additionally loads the model and the raceline).
"""

from __future__ import annotations

import sys

from racer_policy.deploy import ensure_envelope_importable, load_deployment
from racer_policy.errors import ContractError


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print("usage: python3 -m racer_policy.check <policy_dir>", file=sys.stderr)
        return 2
    try:
        from racer_tools.vehicle_params_loader import load_vehicle_params

        ensure_envelope_importable()
        load_deployment(args[0], load_vehicle_params())
    except ContractError as exc:
        print(f"REFUSED {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:  # noqa: BLE001 - anything unexpected is also "not launchable"
        print(f"REFUSED {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
