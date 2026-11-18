"""Telling a starved run from a failed one, for the L5 canaries that drive the real command path.

safety_node times its /drive_raw watchdog and its rate limiter on the wall clock and publishes /drive
on a wall timer, while the simulator's clock never runs faster than the wall. On a loaded CI runner
one process that is not scheduled for a few tens of milliseconds therefore looks, to the stack under
test, like a late command: the watchdog brakes, and the ramp back to speed is clipped by the rate
limit (main run 44199589402: no /drive for 0.34 s while /drive_raw kept its 20 ms cadence, one
watchdog brake and one rate_limit engagement). Nothing about the planner or the gate was wrong, and
whether the machine starves a process is the owner's real-time scheduling decision, not a result of
the code under test.

The evidence that a process was starved is in the run itself: a gap in the planner's /drive_raw or in
safety_node's /drive, both of which are published at 50 Hz in sim time. A run that FAILS its
judgement and has such a gap is run again, at most MAX_ATTEMPTS attempts in all, with the same
judgement every time. A run that fails without such a gap fails at once, and a run that passes is not
looked at. No threshold, gate or assertion is relaxed: a defect that fails the judgement on every
attempt still fails the test, and the output names each starved attempt.
"""

from __future__ import annotations

from collections.abc import Callable

#: A gap (s, sim time) between two /drive_raw or two /drive messages above this means a process was
#: not scheduled. Both streams are 50 Hz, the stamps are on the 10 ms sim step, and a healthy run's
#: largest gap is 30 ms (26 traces of main run 44199589402); 35 ms is the line between three steps
#: and four, so a 40 ms gap is not lost to float rounding.
STARVED_GAP_S = 0.035
MAX_ATTEMPTS = 3


def starvation_gap_s(result: dict) -> float:
    """The longest gap (s) in the planner's /drive_raw or in safety_node's /drive in this run, from
    the runner's `drive_raw` and `safety_drive` summaries (0.0 when it has neither)."""
    return max(
        float(result.get("drive_raw", {}).get("max_gap_s", 0.0)),
        float(result.get("safety_drive", {}).get("max_gap_s", 0.0)),
    )


def run_judged(
    run_once: Callable[[], dict],
    judge: Callable[[dict], None],
    report: Callable[[dict], str],
    say: Callable[[str], None],
    max_attempts: int = MAX_ATTEMPTS,
    starved: Callable[[dict], bool] | None = None,
) -> dict:
    """Run `run_once` and judge its result with `judge` (raises AssertionError). A failing attempt
    that was starved (`starved`, by default a gap over STARVED_GAP_S) is run again until
    `max_attempts` attempts have been made; any other failure is raised at once. Returns the result
    of the attempt that passed."""

    def default_starved(result: dict) -> bool:
        return starvation_gap_s(result) > STARVED_GAP_S

    is_starved = starved or default_starved
    for attempt in range(1, max_attempts + 1):
        result = run_once()
        say(f"{report(result)} (attempt {attempt})")
        try:
            judge(result)
        except AssertionError:
            if attempt == max_attempts or not is_starved(result):
                raise
            say(
                f"attempt {attempt} failed and the machine starved a process "
                f"(longest gap in /drive_raw or /drive {starvation_gap_s(result)} s): run again"
            )
            continue
        return result
    raise AssertionError("max_attempts must be at least 1")
