#!/usr/bin/env bash
# S.6 sim dynamics regression battery (roadmap task S.6, claude-docs/12-testing.md L5
# "Model-upgrade regression"): runs tests/sim_regression's fixed, seeded battery of
# maneuvers against its committed references
# (tests/sim_regression/racer_sim_regression/references/) -- see the sim-regression-battery
# job's path filter in .github/workflows/ci.yml for when this runs.
#
# Same bare-ubuntu-latest + uv pattern as run_python_tests.sh/pytest_gate.sh: racer_gym's
# own pyproject.toml pins f1tenth_gym via a git+commit source, so `uv sync` alone installs
# everything this battery needs -- no docker image required. sim/racer_gym's own tests
# already run this way on every push (run_python_tests.sh's "report" gate); this job
# additionally runs the S.6 battery specifically, gated to pushes that actually touch
# sim/racer_gym/**, tests/sim_regression/**, or tests/replay_harness/** (the golden/
# tolerance engine the battery reuses), since installing and stepping f1tenth_gym through
# several maneuvers is not free and 12-testing.md only requires this on a racer_gym change.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

echo "::group::S.6 sim dynamics regression battery (tests/sim_regression)"
(cd tests/sim_regression && uv sync --all-extras --dev && uv run pytest --cov=. --cov-report=term-missing)
echo "::endgroup::"
