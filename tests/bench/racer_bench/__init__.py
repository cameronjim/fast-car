"""L6 bench/HIL checklist runner scaffold (roadmap task 0.9, claude-docs/12-testing.md).

A procedure is a YAML file of steps (see ``procedures/template_wheels_off_actuation.yaml``
for the shape). Each step is either:

- ``scripted``: a command is run and its exit code (and, optionally,
  stdout content) is checked automatically.
- ``human_confirm``: a human is prompted with a yes/no question and their
  answer is recorded.

:func:`~racer_bench.engine.run_procedure` executes a procedure end to end
and returns a :class:`~racer_bench.engine.SessionRecord`;
:func:`~racer_bench.engine.write_session_record` writes that record to a
timestamped file, because a bench result that only exists in terminal
scrollback did not happen (``claude-docs/12-testing.md``).
"""

from __future__ import annotations
