"""Tests for mux_diag.read_mux_diag's parsing, using the worked example lines from
`docs/notes/mux-diagnostic-build.md` ("Reading the output") as fixtures, so the parser is
checked against the same real formatting code (`firmware/safety_mux/pico/diag_report.c`)
that doc quotes from, not against a hand-imagined format.

Two of that doc's five "real examples of each cut" lines are given there in full (the
nominal PASS line right before them, and the KILLED/RC_KILL_SWITCH cut); those are used
verbatim below. The other three are elided with "..." in the doc (only the field that
changed for that row is spelled out), so a full line for the WATCHDOG_TIMEOUT, STALE, and
OUT_OF_RANGE cases is assembled here from the doc's own verbatim fragments for each field
(the KILL/HB/STEER/THR/OUT segments are all copied byte-for-byte from one or the other of
the two full example lines or from that row's own elided fragment) rather than invented --
see the comment on each one for exactly which fragments came from where.
"""

from __future__ import annotations

import pytest
from mux_diag.read_mux_diag import (
    EXIT_CUT,
    EXIT_PASS,
    exit_code_for,
    format_summary,
    parse_diag_line,
)

# Verbatim from docs/notes/mux-diagnostic-build.md, "A nominal passthrough looks like this:".
NOMINAL_PASS_LINE = (
    "[0 t=12.945s] KILL gp12=1872us FRESH(12ms, in 1000-2000) -> ARMED "
    "(arm>=1600us kill<1400us) | HB gp5 age=11ms (timeout 100ms) OK | "
    "STEER gp10=1500us FRESH(8ms, in 1000-2000) | THR gp7=1500us FRESH(8ms, in 1000-2000) | "
    "DECISION=PASS reason=NORMAL | OUT servo gp1=1500us esc gp3=1500us cutoff gp0=HIGH(power enabled)"
)

# Verbatim from docs/notes/mux-diagnostic-build.md, "Real examples of each cut", line [1] --
# this one is written out in full there, unlike its four siblings.
KILL_SWITCH_CUT_LINE = (
    "[1 t=13.545s] KILL gp12=1102us FRESH(9ms, in 1000-2000) -> KILLED "
    "(arm>=1600us kill<1400us) | HB gp5 age=11ms (timeout 100ms) OK | "
    "STEER gp10=1500us FRESH(8ms, in 1000-2000) | THR gp7=1500us FRESH(8ms, in 1000-2000) | "
    "DECISION=CUT reason=1:RC_KILL_SWITCH | OUT servo gp1=1500us esc gp3=1500us cutoff gp0=LOW(power cut)"
)

# Assembled from the doc's own verbatim fragments for row [3] (elided there as
# "... | DECISION=CUT reason=2:WATCHDOG_TIMEOUT | ..."): KILL and OUT segments copied from
# NOMINAL_PASS_LINE above (row [3]'s own KILL field, "gp12=1872us FRESH(10ms, in
# 1000-2000) -> ARMED", differs from the nominal line only in its age, which this test does
# not depend on), HB and DECISION/reason copied verbatim from the doc's row [3] fragment
# ("HB gp5=NO_EDGES_EVER (timeout 100ms) TIMED_OUT", "reason=2:WATCHDOG_TIMEOUT"), STEER/THR
# copied from NOMINAL_PASS_LINE since row [3] elides them. This is also the exact symptom
# tonight's bench session hit on the real heartbeat pin (see the task this change was made
# for): "HB gp5=NO_EDGES_EVER ... DECISION=CUT reason=2:WATCHDOG_TIMEOUT".
WATCHDOG_TIMEOUT_CUT_LINE = (
    "[3 t=14.745s] KILL gp12=1872us FRESH(10ms, in 1000-2000) -> ARMED "
    "(arm>=1600us kill<1400us) | HB gp5=NO_EDGES_EVER (timeout 100ms) TIMED_OUT | "
    "STEER gp10=1500us FRESH(8ms, in 1000-2000) | THR gp7=1500us FRESH(8ms, in 1000-2000) | "
    "DECISION=CUT reason=2:WATCHDOG_TIMEOUT | OUT servo gp1=1500us esc gp3=1500us cutoff gp0=LOW(power cut)"
)

# Assembled the same way for row [4] (elided as "... | STEER gp10=1500us STALE(last edge
# 430ms ago, window 60ms; stuck or stopped) | ... | DECISION=CUT reason=3:STEERING_PWM_INVALID
# | ..."): STEER and reason copied verbatim from that row's own fragment, everything else
# from NOMINAL_PASS_LINE.
STEERING_STALE_CUT_LINE = (
    "[4 t=15.345s] KILL gp12=1872us FRESH(12ms, in 1000-2000) -> ARMED "
    "(arm>=1600us kill<1400us) | HB gp5 age=11ms (timeout 100ms) OK | "
    "STEER gp10=1500us STALE(last edge 430ms ago, window 60ms; stuck or stopped) | "
    "THR gp7=1500us FRESH(8ms, in 1000-2000) | "
    "DECISION=CUT reason=3:STEERING_PWM_INVALID | OUT servo gp1=1500us esc gp3=1500us cutoff gp0=LOW(power cut)"
)

# Assembled the same way for row [5] (elided as "... | THR gp7=2450us
# OUT_OF_RANGE(allowed 1000-2000) | DECISION=CUT reason=4:THROTTLE_PWM_INVALID | ...").
THROTTLE_OUT_OF_RANGE_CUT_LINE = (
    "[5 t=15.945s] KILL gp12=1872us FRESH(12ms, in 1000-2000) -> ARMED "
    "(arm>=1600us kill<1400us) | HB gp5 age=11ms (timeout 100ms) OK | "
    "STEER gp10=1500us FRESH(8ms, in 1000-2000) | THR gp7=2450us OUT_OF_RANGE(allowed 1000-2000) | "
    "DECISION=CUT reason=4:THROTTLE_PWM_INVALID | OUT servo gp1=1500us esc gp3=1500us cutoff gp0=LOW(power cut)"
)

# The banner and PWMREG lines, verbatim from the same doc, must not be mistaken for a
# decision line.
BANNER_LINES = [
    (
        "=== safety_mux DIAGNOSTIC build (printing only; decision logic identical to the "
        "shipping build) ==="
    ),
    "pins: kill=gp12 steer=gp10 throttle=gp7 heartbeat=gp5 | out servo=gp1 esc=gp3 cutoff=gp0",
    (
        "params: steer 1000-2000 (neutral 1500) | throttle 1000-2000 (neutral 1500) | "
        "watchdog 100ms | kill thr 1500us +-100us | rc range 1000-2000"
    ),
    "cut priority: 1 rc kill/unreadable, 2 heartbeat watchdog, 3 steering, 4 throttle",
]
PWMREG_LINE = (
    "    PWMREG clk_sys=125000000Hz | servo gp1 slice0.B top=39999 div=62.500 level=3000 "
    "en=1 pinfn=PWM -> pulse=1500.0us frame=20000.0us 50.00Hz duty=7.50% | "
    "esc gp3 slice1.B top=39999 div=62.500 level=3000 en=1 pinfn=PWM -> pulse=1500.0us "
    "frame=20000.0us 50.00Hz duty=7.50%"
)


def test_nominal_pass_parses():
    report = parse_diag_line(NOMINAL_PASS_LINE)
    assert report is not None
    assert report.seq == 0
    assert report.t_s == pytest.approx(12.945)
    assert report.kill_position == "ARMED"
    assert report.kill.pulse_us == 1872
    assert report.kill.status == "FRESH"
    assert report.heartbeat.status == "OK"
    assert report.heartbeat.age_ms == 11
    assert report.steer.pulse_us == 1500
    assert report.throttle.pulse_us == 1500
    assert report.decision == "PASS"
    assert report.reason == "NORMAL"
    assert exit_code_for(report) == EXIT_PASS
    assert format_summary(report) == (
        "KILL ARMED 1872us | HB OK age 11ms | STEER 1500us | THR 1500us | DECISION PASS"
    )


def test_kill_switch_cut_parses():
    report = parse_diag_line(KILL_SWITCH_CUT_LINE)
    assert report is not None
    assert report.kill_position == "KILLED"
    assert report.kill.pulse_us == 1102
    assert report.decision == "CUT"
    assert report.reason == "1:RC_KILL_SWITCH"
    assert exit_code_for(report) == EXIT_CUT
    assert format_summary(report) == (
        "KILL KILLED 1102us | HB OK age 11ms | STEER 1500us | THR 1500us | "
        "DECISION CUT reason 1:RC_KILL_SWITCH"
    )


def test_watchdog_timeout_cut_parses():
    """This is tonight's actual bench symptom: heartbeat never seen, mux cuts."""
    report = parse_diag_line(WATCHDOG_TIMEOUT_CUT_LINE)
    assert report is not None
    assert report.heartbeat.status == "TIMED_OUT"
    assert report.heartbeat.age_ms is None
    assert report.decision == "CUT"
    assert report.reason == "2:WATCHDOG_TIMEOUT"
    assert exit_code_for(report) == EXIT_CUT
    assert format_summary(report) == (
        "KILL ARMED 1872us | HB NO_EDGES_EVER | STEER 1500us | THR 1500us | "
        "DECISION CUT reason 2:WATCHDOG_TIMEOUT"
    )


def test_steering_stale_cut_parses():
    report = parse_diag_line(STEERING_STALE_CUT_LINE)
    assert report is not None
    assert report.steer.status == "STALE"
    assert report.steer.pulse_us == 1500  # STALE still carries the last-known width
    assert report.reason == "3:STEERING_PWM_INVALID"
    assert exit_code_for(report) == EXIT_CUT
    assert format_summary(report) == (
        "KILL ARMED 1872us | HB OK age 11ms | STEER 1500us | THR 1500us | "
        "DECISION CUT reason 3:STEERING_PWM_INVALID"
    )


def test_throttle_out_of_range_cut_parses():
    report = parse_diag_line(THROTTLE_OUT_OF_RANGE_CUT_LINE)
    assert report is not None
    assert report.throttle.status == "OUT_OF_RANGE"
    assert report.throttle.pulse_us == 2450
    assert report.reason == "4:THROTTLE_PWM_INVALID"
    assert exit_code_for(report) == EXIT_CUT
    assert format_summary(report) == (
        "KILL ARMED 1872us | HB OK age 11ms | STEER 1500us | THR 2450us | "
        "DECISION CUT reason 4:THROTTLE_PWM_INVALID"
    )


@pytest.mark.parametrize("line", BANNER_LINES + [PWMREG_LINE, "", "garbage\n"])
def test_non_decision_lines_are_not_a_decision(line):
    assert parse_diag_line(line) is None


def test_kill_no_edges_has_no_pulse_width():
    # "gp12=--- NO_EDGES(...)" fragment, verbatim from
    # docs/notes/mux-diagnostic-build.md row [2] (the rest of that row is elided there, so
    # only the KILL field itself -- the one this test checks -- is taken from the doc).
    line = (
        "[2 t=14.145s] KILL gp12=--- NO_EDGES(no plausible pulse since boot; "
        "line dead/unplugged/noise) -> UNREADABLE (arm>=1600us kill<1400us) | "
        "HB gp5 age=11ms (timeout 100ms) OK | STEER gp10=1500us FRESH(8ms, in 1000-2000) | "
        "THR gp7=1500us FRESH(8ms, in 1000-2000) | DECISION=CUT reason=1:RC_SIGNAL_INVALID | "
        "OUT servo gp1=1500us esc gp3=1500us cutoff gp0=LOW(power cut)"
    )
    report = parse_diag_line(line)
    assert report is not None
    assert report.kill_position == "UNREADABLE"
    assert report.kill.pulse_us is None
    assert report.kill.status == "NO_EDGES"
    assert format_summary(report) == (
        "KILL UNREADABLE NO_EDGES | HB OK age 11ms | STEER 1500us | THR 1500us | "
        "DECISION CUT reason 1:RC_SIGNAL_INVALID"
    )
