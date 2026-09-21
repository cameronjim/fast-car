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
    _LineBuffer,
    exit_code_for,
    format_summary,
    parse_diag_line,
    run_once,
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


# --- _LineBuffer: partial-first-line discard -------------------------------------------
#
# This exercises the fix for the bench bug where a one-shot `read_mux_diag.py` invocation
# could hang and hold `/dev/ttyACM0` open: the reader attaches to an already-running
# diagnostic firmware mid-stream, so the first bytes it ever reads can be the tail end of a
# line that started before the fd was opened. `_LineBuffer` is exercised directly here
# (a fake byte stream, no real serial device) because there is no Pico USB port to open on
# this host -- the real device path (`_SerialLineReader` against an actual tty) stays
# untested.


def test_line_buffer_discards_partial_first_line():
    buf = _LineBuffer()
    # "us) | DECISION..." looks like the tail of a line already in flight when we attached;
    # it is followed by a newline and then one complete, real line.
    buf.feed(b"us) | DECISION=PASS reason=NORMAL | tail of a line we joined mid-stream\n")
    buf.feed(NOMINAL_PASS_LINE.encode() + b"\n")

    line = buf.pop_line()

    assert line == NOMINAL_PASS_LINE


def test_line_buffer_discards_partial_first_line_fed_in_pieces():
    buf = _LineBuffer()
    # The partial-first-line fragment arrives split across several small reads.
    for piece in (b"partial", b" fragment", b" before", b" our", b" first", b" newline\n"):
        buf.feed(piece)
        assert buf.pop_line() is None  # nothing complete yet; still mid-fragment or discarded
    buf.feed(KILL_SWITCH_CUT_LINE.encode() + b"\n")

    assert buf.pop_line() == KILL_SWITCH_CUT_LINE


def test_line_buffer_returns_subsequent_full_lines_in_order():
    buf = _LineBuffer()
    buf.feed(
        b"discarded partial\n"
        + NOMINAL_PASS_LINE.encode()
        + b"\n"
        + KILL_SWITCH_CUT_LINE.encode()
        + b"\n"
    )

    assert buf.pop_line() == NOMINAL_PASS_LINE
    assert buf.pop_line() == KILL_SWITCH_CUT_LINE
    assert buf.pop_line() is None


def test_line_buffer_no_discard_needed_when_buffer_starts_empty_and_stays_synced():
    """Once synced (the first newline has been seen), every later line is returned as-is --
    the discard applies only to the very first line, never to normal steady-state reads."""
    buf = _LineBuffer()
    buf.feed(b"\n")  # an empty first line: discarded same as any other first line
    assert buf.pop_line() is None
    buf.feed(NOMINAL_PASS_LINE.encode() + b"\n")
    assert buf.pop_line() == NOMINAL_PASS_LINE


# --- run_once: stops at the first parseable verdict line --------------------------------


class _FakeReader:
    """A fake `_SerialLineReader` for `run_once`: yields lines from a fixed list, one per
    `readline()` call, and records how many times it was called so a test can assert
    `run_once` stopped as soon as it found a parseable line instead of reading everything
    available (the other half of the same bench bug: consecutive one-shot calls each
    taking ~5s because the old default kept reading up to a high line count regardless).
    """

    def __init__(self, lines: list[str | None]):
        self._lines = list(lines)
        self.calls = 0

    def readline(self, timeout_s: float) -> str | None:
        self.calls += 1
        if not self._lines:
            return None
        return self._lines.pop(0)


def test_run_once_stops_at_first_parseable_line():
    reader = _FakeReader([PWMREG_LINE, NOMINAL_PASS_LINE, KILL_SWITCH_CUT_LINE])

    report = run_once(reader, max_lines=10, timeout_s=5.0)

    assert report is not None
    assert report.decision == "PASS"
    assert reader.calls == 2  # PWMREG (skipped), then the first real verdict line


def test_run_once_returns_none_when_lines_exhausted_without_a_verdict():
    reader = _FakeReader(BANNER_LINES + [PWMREG_LINE])

    report = run_once(reader, max_lines=len(BANNER_LINES) + 1, timeout_s=5.0)

    assert report is None


def test_run_once_returns_none_when_reader_yields_nothing():
    """Models a timed-out `readline()` (nothing arrived before the deadline)."""
    reader = _FakeReader([None])

    report = run_once(reader, max_lines=10, timeout_s=5.0)

    assert report is None
    assert reader.calls == 1
