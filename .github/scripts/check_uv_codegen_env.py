#!/usr/bin/env python3
"""Every `uv run --project tools` codegen step must run with PYTHONPATH unset.

Regression guard for the bug found during the first real Jetson bring-up on 2026-09-21
(docs/notes/build-log.md). `tools/` is a uv-managed project with its own interpreter
(CPython 3.14 at the time of writing). An inherited PYTHONPATH from the surrounding
environment is prepended to that interpreter's sys.path, AHEAD of the project's own
site-packages, so a foreign site-packages directory wins the import. On the Jetson, where
docker/car sets `ENV PYTHONPATH=/car-runtime/.venv/lib/python3.10/site-packages` for
racer_policy's runtime deps, `colcon build` died in the vehicle_params codegen step with
`ModuleNotFoundError: No module named 'rpds.rpds'` -- a cp310 native extension being loaded
by a 3.14 interpreter.

`docker/ros-dev` is one `export PYTHONPATH=...` away from the same failure (see
docs/notes/milestone-5-browser-teleop.md's demo procedure, which does exactly that before
`colcon build`), which is why the fix is `cmake -E env --unset=PYTHONPATH` in each
CMakeLists.txt rather than an incantation in the car image's run command.

Run by the `lint` CI job. No Docker, no ROS, no third-party imports.
"""

from __future__ import annotations

import pathlib
import re
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
CMAKE_FILES = sorted((REPO_ROOT / "ros_ws/src").glob("*/CMakeLists.txt"))

# The uv invocation, and the wrapper that must immediately precede it on the COMMAND line.
UV_CALL = re.compile(r'"\$\{RACER_UV_EXECUTABLE\}"\s+run\s+--project')
GUARD = re.compile(r'COMMAND\s+"\$\{CMAKE_COMMAND\}"\s+-E\s+env\s+--unset=PYTHONPATH\s+')


def main() -> int:
    problems: list[str] = []
    checked = 0

    for path in CMAKE_FILES:
        text = path.read_text()
        for match in UV_CALL.finditer(text):
            checked += 1
            # The guard must be the COMMAND directive introducing this uv call, i.e. it ends
            # exactly where the uv executable begins (modulo the line break and indentation
            # that `\s+` already absorbs).
            preceding = text[: match.start()]
            if not GUARD.search(preceding[-400:]) or not re.search(
                GUARD.pattern + r"$", preceding, re.MULTILINE | re.DOTALL
            ):
                problems.append(
                    f"{path.relative_to(REPO_ROOT)}: `uv run --project` is invoked without a "
                    'preceding `COMMAND "${CMAKE_COMMAND}" -E env --unset=PYTHONPATH`. An '
                    "inherited PYTHONPATH shadows the tools/ venv's own site-packages and "
                    "breaks this codegen step (docker/car sets one; see this script's "
                    "docstring)."
                )

    if not checked:
        print(
            "ERROR: found no `uv run --project` codegen steps in ros_ws/src/*/CMakeLists.txt. "
            "Either the pattern moved (update this check) or the check is now vacuous.",
            file=sys.stderr,
        )
        return 1

    if problems:
        for problem in problems:
            print(f"ERROR: {problem}", file=sys.stderr)
        return 1

    print(f"uv codegen PYTHONPATH guard OK ({checked} invocation(s) checked)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
