#!/usr/bin/env bash
# L4-small (replay/golden) and L5-short (headless sim-in-loop) per
# claude-docs/12-testing.md "CI wiring": both are supposed to run on every
# push.
#
# Roadmap task 0.9 landed the harness *frameworks*
# (tests/replay_harness, tests/sim_in_loop, tests/bench) but not yet any
# real content to run them on: no curated rosbags exist (tests/bags/ is
# still empty -- they arrive with hardware, roadmap task 2.8), and no real
# tracker/reference track exists yet for a real lap test (roadmap task
# S.2). So there is no real L4/L5 *pipeline* run to wire in here yet.
#
# What this script runs instead, as the most honest stand-in available
# right now: each harness's own self-tests (run_python_tests.sh already
# runs these too, via pytest_gate.sh "report" gate -- this is deliberately
# redundant with that, not a replacement for it, so that L4-small/L5-short
# stays green/red independent of whether someone reshuffles
# run_python_tests.sh's package list). This is the harness working, not a
# replay/sim-in-loop result -- whoever lands task 2.8 (real bags) or S.2
# (tracker lap test) should replace the calls below with the real
# runner invocations against real fixtures/tracks.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

export PATH="${HOME}/.local/bin:${PATH}"

BAG_FILES="$(find tests/bags -type f -not -name '.gitkeep' 2>/dev/null || true)"
if [ -n "$BAG_FILES" ]; then
  echo "NOTICE: tests/bags/ has content, but no L4 replay runner is wired yet (task 2.8)."
else
  echo "NOTICE: tests/bags/ is empty. No L4 replay fixtures yet (task 2.8)."
fi

echo "::group::L4-small stand-in: tests/replay_harness self-tests (golden engine + fault injectors)"
(cd tests/replay_harness && uv sync --all-extras --dev && uv run pytest --cov=. --cov-report=term-missing)
echo "::endgroup::"

echo "::group::L5-short stand-in: tests/sim_in_loop self-tests (fake-env plumbing; gym-backed test skips without f1tenth_gym installed)"
(cd tests/sim_in_loop && uv sync --all-extras --dev && uv run pytest --cov=. --cov-report=term-missing)
echo "::endgroup::"

echo "NOTICE: no real tracker/reference track exists yet for a real L5 lap test (task S.2)."
echo "L4-small/L5-short harness self-tests passed. Real replay/sim-in-loop runs land with tasks 2.8/S.2."
