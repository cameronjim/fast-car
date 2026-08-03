#!/usr/bin/env bash
# Placeholder for L4-small (replay/golden tests on small committed rosbags)
# and L5-short (headless sim-in-loop, short) per claude-docs/12-testing.md
# "CI wiring": both are supposed to run on every push. The real harnesses
# land in:
#   - roadmap task 0.9: replay/golden framework + bag-mutation fault
#     injectors (L4), sim-in-loop runner (L5), bench checklist runner (L6)
#   - roadmap task S.2: tracker lap test committed as the CI regression
#     canary (L5)
#
# Until those tasks land, this step only looks for their entry points and
# always passes with a loud notice - it is not a real test gate yet, and it
# must not become a silent no-op once 0.9/S.2 add real content: whoever
# lands those tasks needs to replace this script's body with the real L4/L5
# runner invocations, not just leave it printing notices forever.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

BAG_FILES="$(find tests/bags -type f -not -name '.gitkeep' 2>/dev/null || true)"
if [ -n "$BAG_FILES" ]; then
  echo "NOTICE: tests/bags/ has content, but no L4 replay runner is wired yet (task 0.9)."
else
  echo "NOTICE: tests/bags/ is empty. No L4 replay fixtures yet (task 0.9)."
fi

SIM_ENTRY_POINTS="$(find sim tests -type f \( -iname '*sim_in_loop*' -o -iname '*tracker_lap*' \) 2>/dev/null || true)"
if [ -n "$SIM_ENTRY_POINTS" ]; then
  echo "NOTICE: possible L5 sim-in-loop entry point(s) found, but no runner is wired yet (task 0.9/S.2)."
else
  echo "NOTICE: no L5 sim-in-loop entry points yet (task 0.9/S.2)."
fi

echo "NOTICE: L4-small and L5-short are placeholders until tasks 0.9 and S.2 land. Passing."
exit 0
