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

`tools/mux_diag/tests/fixtures/mux_raw_2026-09-21.txt` is a separate fixture: a verbatim
raw capture from the real Pico on the bench, used near the end of this file to reproduce
and fix the 2026-09-21 regression where `read_mux_diag.py` failed against the live device
even though the raw serial stream itself was fine.
"""

from __future__ import annotations

import os
import pty
import threading
import tty
from pathlib import Path

import pytest
from mux_diag.read_mux_diag import (
    DEFAULT_LINES,
    DEFAULT_TIMEOUT_S,
    EXIT_CUT,
    EXIT_PASS,
    _LineBuffer,
    _SerialLineReader,
    exit_code_for,
    format_summary,
    parse_diag_line,
    run_once,
)

FIXTURES_DIR = Path(__file__).parent / "fixtures"

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


# --- run_once: `max_lines` counts parseable verdict lines, never raw serial lines -------
#
# This is the regression fix itself. Before it, `max_lines` bounded how many raw serial
# lines could be read at all, counting the attach banner, a discarded partial first line,
# and stray PWMREG-only fragments the same as a real verdict line. With the shipped
# default of 1 raw line, a fresh attach (banner first, verdict lines after) almost always
# spent that single allowed read on banner text and reported "no parseable mux diagnostic
# line", even though good verdict lines were arriving right behind it -- see
# `test_run_once_survives_a_full_attach_banner_before_the_first_verdict` below and the
# fixture-driven test using the real captured bytes.


class _FakeReader:
    """A fake `_SerialLineReader` for `run_once`: yields lines from a fixed list, one per
    `readline()` call, and records how many times it was called so a test can assert how
    many raw reads a given scenario took.
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

    report = run_once(reader, max_lines=1, timeout_s=5.0)

    assert report is not None
    assert report.decision == "PASS"
    assert reader.calls == 2  # PWMREG (skipped), then the first real verdict line


def test_run_once_survives_a_full_attach_banner_before_the_first_verdict():
    """The exact shape of the regression: `max_lines=1` (the default) must not be spent on
    the four-line attach banner that precedes every real verdict line on a fresh attach."""
    reader = _FakeReader([*BANNER_LINES, PWMREG_LINE, NOMINAL_PASS_LINE])

    report = run_once(reader, max_lines=1, timeout_s=5.0)

    assert report is not None
    assert report.decision == "PASS"
    assert reader.calls == len(BANNER_LINES) + 2  # banner + PWMREG all skipped, then PASS


def test_run_once_with_max_lines_above_one_waits_for_that_many_verdicts():
    reader = _FakeReader([NOMINAL_PASS_LINE, PWMREG_LINE, KILL_SWITCH_CUT_LINE])

    report = run_once(reader, max_lines=2, timeout_s=5.0)

    # Returns the *second* parseable verdict line seen, per `max_lines=2`, not the first.
    assert report is not None
    assert report.decision == "CUT"
    assert reader.calls == 3


def test_run_once_returns_none_when_reader_runs_dry_without_a_verdict():
    """Bounded by the reader running out of data (EOF/disconnect), never by a raw line
    count -- junk chatter alone must never trip an early give-up."""
    reader = _FakeReader(BANNER_LINES + [PWMREG_LINE])

    report = run_once(reader, max_lines=1, timeout_s=5.0)

    assert report is None
    assert reader.calls == len(BANNER_LINES) + 1 + 1  # all junk, then the None that ends it


def test_run_once_returns_none_when_reader_yields_nothing():
    """Models a timed-out `readline()` (nothing arrived before the deadline)."""
    reader = _FakeReader([None])

    report = run_once(reader, max_lines=1, timeout_s=5.0)

    assert report is None
    assert reader.calls == 1


# --- Fixture-driven regression test against the real device path (a pty, not a fake) ----
#
# `tools/mux_diag/tests/fixtures/mux_raw_2026-09-21.txt` is a verbatim 15-line raw capture
# from the actual mux Pico on the bench (`stty -F /dev/ttyACM0 115200 raw -echo; timeout 5
# cat /dev/ttyACM0`), taken the same session `read_mux_diag.py` failed against the live
# device with "timed out ... no parseable mux diagnostic line" while the raw `stty`/`cat`
# capture read perfectly good lines. It is CRLF-terminated and starts with a stray blank
# `\r\n` (a partial/junk fragment already in flight when the capture attached), then the
# four-line attach banner, then ten real verdict-line reports -- exactly the shape that
# broke `run_once` when its budget counted raw lines (banner included) instead of
# parseable ones. This test drives the actual `_SerialLineReader` (open/select/os.read)
# against a pty loaded with those exact bytes, rather than only the pure `_LineBuffer`
# logic, so the fix is checked against the same code path the real device uses -- that
# path was previously untested on this host because there is no Pico USB port to open
# here (see `_LineBuffer`'s docstring).


def _feed_nonblocking(fd: int, data: bytes, stop: threading.Event) -> None:
    """Writes `data` to `fd` (opened `O_NONBLOCK`), retrying past `BlockingIOError` (the
    pty's internal buffer is smaller than this fixture) until it is all written, `stop` is
    set, or the fd errors out from under this thread (`OSError`).

    Run on a background thread against the pty's master fd while the main thread reads
    from the slave side: a single blocking `os.write` big enough to fill the pty's buffer
    would otherwise block forever waiting for a reader that only shows up afterwards.
    Non-blocking writes plus a short sleep on `BlockingIOError`, instead, let the test
    close the fds and move on the moment it has read what it needs, without either side
    ever having to fully drain the fixture. `stop` (checked by the caller via `.join()`
    after setting it) matters beyond tidiness: fd numbers get reused as soon as they are
    closed, so a still-running writer left over from a previous test could otherwise write
    stale fixture bytes into a *later* test's freshly opened, unrelated pty.
    """
    view = memoryview(data)
    while view and not stop.is_set():
        try:
            n = os.write(fd, view)
            view = view[n:]
        except BlockingIOError:
            stop.wait(0.005)
        except OSError:
            return


def _open_reader_on_pty_with_fixture(
    fixture_name: str,
) -> tuple[_SerialLineReader, int, int, threading.Event, threading.Thread]:
    """Opens a `_SerialLineReader` on the slave end of a fresh pty, feeding the given
    fixture file's raw bytes in from the master end on a background thread, so this
    mirrors a real device streaming data to `/dev/ttyACM0` while the reader reads it.

    The original slave fd is kept open for the pty's lifetime (writing to the master with
    no open slave end raises `OSError: [Errno 5]`); `_SerialLineReader` opens its own,
    second fd on the same slave path, exactly as it would open a real `/dev/ttyACM0`.

    A fresh pty also defaults to canonical mode with echo, like an interactive terminal,
    not like a real serial device; `tty.setraw` matches the bench's own `stty ... raw
    -echo` and how a real `/dev/ttyACM0` behaves.

    Returns the reader, the master and slave fds, and the feeder's stop event and thread.
    The caller's `finally` must, in order: set the stop event, `.join()` the thread, then
    `os.close()` both fds (alongside `reader.close()`) -- see `_feed_nonblocking` for why
    the join has to happen before the close.
    """
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    os.set_blocking(master_fd, False)
    slave_path = os.ttyname(slave_fd)
    data = (FIXTURES_DIR / fixture_name).read_bytes()
    stop = threading.Event()
    writer = threading.Thread(target=_feed_nonblocking, args=(master_fd, data, stop), daemon=True)
    writer.start()
    reader = _SerialLineReader(slave_path)
    return reader, master_fd, slave_fd, stop, writer


def test_real_device_path_survives_the_captured_attach_banner_and_finds_a_verdict():
    reader, master_fd, slave_fd, stop, writer = _open_reader_on_pty_with_fixture(
        "mux_raw_2026-09-21.txt"
    )
    try:
        report = run_once(reader, max_lines=DEFAULT_LINES, timeout_s=DEFAULT_TIMEOUT_S)

        assert report is not None
        # The fixture's first real verdict line, report [2]: KILL KILLED (RC_KILL_SWITCH).
        assert report.decision == "CUT"
        assert report.reason == "1:RC_KILL_SWITCH"
        assert report.kill_position == "KILLED"
    finally:
        reader.close()
        stop.set()
        writer.join(timeout=5)
        os.close(master_fd)
        os.close(slave_fd)


def test_real_device_path_can_find_a_later_report_too():
    """Report [9] in the fixture is the one OUT_OF_RANGE/RC_SIGNAL_INVALID cut among the
    otherwise-identical KILL_SWITCH cuts -- picking it out proves this is reading forward
    through the real stream, not just latching onto the first report found."""
    reader, master_fd, slave_fd, stop, writer = _open_reader_on_pty_with_fixture(
        "mux_raw_2026-09-21.txt"
    )
    try:
        # Reports [2]..[9] inclusive is 8 verdict lines; [9] is the 8th one.
        report = run_once(reader, max_lines=8, timeout_s=DEFAULT_TIMEOUT_S)

        assert report is not None
        assert report.reason == "1:RC_SIGNAL_INVALID"
        assert report.kill_position == "UNREADABLE"
    finally:
        reader.close()
        stop.set()
        writer.join(timeout=5)
        os.close(master_fd)
        os.close(slave_fd)
