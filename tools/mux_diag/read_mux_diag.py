"""Reads and summarizes the safety_mux DIAGNOSTIC firmware's serial output.

Bench tool (roadmap task 1.3, layer-1 safety mux, `claude-docs/05-safety.md`). The
diagnostic firmware (`docs/notes/mux-diagnostic-build.md`, opt-in `DIAG_BUILD=ON`, never
the shipping build) prints one banner on attach and then, about twice a second, one line
per control cycle describing what the mux saw and decided, plus a second `PWMREG` line this
tool ignores. The exact format is produced by `firmware/safety_mux/pico/diag_report.c`'s
`printf`/`snprintf` calls; this module's regexes are written against that source, not
against the doc's prose, and the doc's own worked examples
(`docs/notes/mux-diagnostic-build.md` "Reading the output") are this package's test fixtures.

This is a bench debugging aid, same as the firmware it reads: it takes no action on the car
and changes no safety behavior. It is Python 3 stdlib only (`os`/`select` for a
non-blocking read with a timeout, no `pyserial`), because a bench script for diagnosing "the
heartbeat never reaches the mux" should not itself need a package install to run.

Usage:
    python -m mux_diag.read_mux_diag                       # one summary from /dev/ttyACM0
    python -m mux_diag.read_mux_diag --device /dev/ttyACM1
    python -m mux_diag.read_mux_diag --watch                # keep printing, like tail -f
    python -m mux_diag.read_mux_diag --json

Exit codes: 0 DECISION=PASS, 2 DECISION=CUT, 3 no diagnostic line was seen (bad device, wrong
firmware flashed, nothing plugged into USB, or the read window/timeout ran out first).
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import re
import select
import sys
import time

DEFAULT_DEVICE = "/dev/ttyACM0"
DEFAULT_LINES = 1
DEFAULT_TIMEOUT_S = 5.0

EXIT_PASS = 0
EXIT_CUT = 2
EXIT_NO_DATA = 3

# One data line looks like:
#   [0 t=12.945s] KILL gp12=1872us FRESH(12ms, in 1000-2000) -> ARMED (arm>=1600us
#   kill<1400us) | HB gp5 age=11ms (timeout 100ms) OK | STEER gp10=1500us FRESH(8ms, in
#   1000-2000) | THR gp7=1500us FRESH(8ms, in 1000-2000) | DECISION=PASS reason=NORMAL |
#   OUT servo gp1=1500us esc gp3=1500us cutoff gp0=HIGH(power enabled)
# Everything else the firmware prints (the four-line attach banner, the "PWMREG" line) does
# not start with "[<seq> t=" and is skipped rather than matched.
_LINE_RE = re.compile(r"^\[(?P<seq>\d+)\s+t=(?P<t_s>[\d.]+)s\]\s*(?P<rest>.*)$")

# One capture channel (KILL/STEER/THR), e.g. "gp12=1872us FRESH(12ms, in 1000-2000)",
# "gp12=--- NO_EDGES(...)", "gp10=1500us STALE(...)", "gp7=2450us OUT_OF_RANGE(...)", or
# "gp3=?? NOT_REGISTERED" -- see diag_report.c's format_channel(). The "us" suffix is only
# present when the value is a real pulse width, never on "---" or "??".
_CHANNEL_RE = re.compile(r"^gp(?P<gpio>\d+)=(?P<val>---|\?\?|\d+)(?:us)?\s+(?P<status>[A-Z_]+)")

# "HB gp5 age=11ms (timeout 100ms) OK" or "HB gp5=NO_EDGES_EVER (timeout 100ms) TIMED_OUT" --
# see diag_report.c's two snprintf calls for hb_s.
_HB_RE = re.compile(
    r"^HB gp(?P<gpio>\d+)(?:=NO_EDGES_EVER|\s+age=(?P<age_ms>\d+)ms)\s+"
    r"\(timeout (?P<timeout_ms>\d+)ms\)\s+(?P<status>OK|TIMED_OUT)$"
)

_DECISION_RE = re.compile(r"^DECISION=(?P<decision>PASS|CUT)\s+reason=(?P<reason>\S+)$")


@dataclasses.dataclass(frozen=True)
class Channel:
    gpio: int
    pulse_us: int | None  # None for "---" (NO_EDGES) or "??" (NOT_REGISTERED)
    status: str  # FRESH, NO_EDGES, STALE, OUT_OF_RANGE, NOT_REGISTERED


@dataclasses.dataclass(frozen=True)
class Heartbeat:
    gpio: int
    status: str  # OK or TIMED_OUT
    age_ms: int | None  # None means NO_EDGES_EVER
    timeout_ms: int


@dataclasses.dataclass(frozen=True)
class DiagReport:
    seq: int
    t_s: float
    kill: Channel
    kill_position: str  # ARMED, KILLED, UNREADABLE
    heartbeat: Heartbeat
    steer: Channel
    throttle: Channel
    decision: str  # PASS or CUT
    reason: str  # NORMAL, or "<n>:REASON_NAME"


def _parse_channel(text: str) -> Channel | None:
    m = _CHANNEL_RE.match(text.strip())
    if m is None:
        return None
    val = m.group("val")
    pulse_us = int(val) if val.isdigit() else None
    return Channel(gpio=int(m.group("gpio")), pulse_us=pulse_us, status=m.group("status"))


def parse_diag_line(line: str) -> DiagReport | None:
    """Parses one diagnostic report line, or returns None for anything else.

    "Anything else" includes the attach banner, the PWMREG line, blank lines, and any line
    this parser cannot confidently make sense of -- callers should treat None as "not a
    decision line", never raise on unrecognized bench chatter.
    """
    m = _LINE_RE.match(line.rstrip("\r\n"))
    if m is None:
        return None

    parts = [p.strip() for p in m.group("rest").split(" | ")]
    if len(parts) < 5:
        return None
    kill_part, hb_part, steer_part, thr_part, decision_part = parts[:5]

    if not kill_part.startswith("KILL "):
        return None
    kill_rest = kill_part[len("KILL ") :]
    if " -> " not in kill_rest:
        return None
    kill_channel_text, kill_tail = kill_rest.split(" -> ", 1)
    kill_position = kill_tail.split(" ", 1)[0]
    if kill_position not in ("ARMED", "KILLED", "UNREADABLE"):
        return None
    kill = _parse_channel(kill_channel_text)
    if kill is None:
        return None

    hb_m = _HB_RE.match(hb_part)
    if hb_m is None:
        return None
    age_ms = hb_m.group("age_ms")
    heartbeat = Heartbeat(
        gpio=int(hb_m.group("gpio")),
        status=hb_m.group("status"),
        age_ms=int(age_ms) if age_ms is not None else None,
        timeout_ms=int(hb_m.group("timeout_ms")),
    )

    if not steer_part.startswith("STEER "):
        return None
    steer = _parse_channel(steer_part[len("STEER ") :])
    if steer is None:
        return None

    if not thr_part.startswith("THR "):
        return None
    throttle = _parse_channel(thr_part[len("THR ") :])
    if throttle is None:
        return None

    decision_m = _DECISION_RE.match(decision_part)
    if decision_m is None:
        return None

    return DiagReport(
        seq=int(m.group("seq")),
        t_s=float(m.group("t_s")),
        kill=kill,
        kill_position=kill_position,
        heartbeat=heartbeat,
        steer=steer,
        throttle=throttle,
        decision=decision_m.group("decision"),
        reason=decision_m.group("reason"),
    )


def _channel_str(channel: Channel) -> str:
    if channel.pulse_us is not None:
        return f"{channel.pulse_us}us"
    return channel.status


def format_summary(report: DiagReport) -> str:
    """The one-line human summary, e.g.:

    "KILL ARMED 2000us | HB OK age 11ms | STEER 1484us | THR 1484us | DECISION PASS"
    "KILL ARMED 1872us | HB NO_EDGES_EVER | STEER 1500us | THR 1500us | DECISION CUT
        reason 2:WATCHDOG_TIMEOUT"
    """
    kill_str = f"{report.kill_position} {_channel_str(report.kill)}"

    if report.heartbeat.age_ms is None:
        hb_str = "NO_EDGES_EVER"
    else:
        hb_str = f"{report.heartbeat.status} age {report.heartbeat.age_ms}ms"

    steer_str = _channel_str(report.steer)
    thr_str = _channel_str(report.throttle)

    if report.decision == "PASS":
        decision_str = "PASS"
    else:
        decision_str = f"CUT reason {report.reason}"

    return (
        f"KILL {kill_str} | HB {hb_str} | STEER {steer_str} | THR {thr_str} | "
        f"DECISION {decision_str}"
    )


def exit_code_for(report: DiagReport) -> int:
    return EXIT_PASS if report.decision == "PASS" else EXIT_CUT


def report_to_dict(report: DiagReport) -> dict:
    return {
        "seq": report.seq,
        "t_s": report.t_s,
        "kill": {
            "gpio": report.kill.gpio,
            "pulse_us": report.kill.pulse_us,
            "status": report.kill.status,
            "position": report.kill_position,
        },
        "heartbeat": {
            "gpio": report.heartbeat.gpio,
            "status": report.heartbeat.status,
            "age_ms": report.heartbeat.age_ms,
            "timeout_ms": report.heartbeat.timeout_ms,
        },
        "steer": {
            "gpio": report.steer.gpio,
            "pulse_us": report.steer.pulse_us,
            "status": report.steer.status,
        },
        "throttle": {
            "gpio": report.throttle.gpio,
            "pulse_us": report.throttle.pulse_us,
            "status": report.throttle.status,
        },
        "decision": report.decision,
        "reason": report.reason,
        "summary": format_summary(report),
        "exit_code": exit_code_for(report),
    }


class _LineBuffer:
    """Buffers raw bytes fed to it into complete newline-terminated lines.

    Pure and I/O-free on purpose, so the partial-first-line handling below can be tested
    against a fake sequence of chunks instead of a real serial device (the real device path
    stays untested on the host -- there is no Pico USB port to open here).

    The device is a running, already-attached diagnostic firmware (`--watch` sessions and
    prior invocations may have been reading it for a while); when this process opens the
    fd, whatever the kernel already has buffered can start mid-line. The first line ever
    popped is therefore always discarded rather than returned, on the assumption it may be
    a partial fragment of a line that started before this reader existed -- see
    `docs/notes/mux-diagnostic-build.md`'s "Not blocking on USB" for why lines can also be
    dropped mid-stream by the firmware itself, which this same discard-and-resync behavior
    tolerates.
    """

    def __init__(self) -> None:
        self._buf = b""
        self._synced = False

    def feed(self, chunk: bytes) -> None:
        self._buf += chunk

    def pop_line(self) -> str | None:
        """Returns and consumes one complete line, or None if none is buffered yet."""
        while True:
            newline_at = self._buf.find(b"\n")
            if newline_at == -1:
                return None
            line = self._buf[:newline_at]
            self._buf = self._buf[newline_at + 1 :]
            if not self._synced:
                # This first line may have started before we ever opened the device.
                # Discard it and resync on the next newline instead of returning it.
                self._synced = True
                continue
            return line.decode("utf-8", errors="replace")


class _SerialLineReader:
    """Non-blocking, timeout-bounded line reader for a tty character device.

    Opens the device read-only, non-blocking (`O_NONBLOCK`), so a read with nothing waiting
    returns immediately instead of hanging forever -- the mux's USB CDC port ignores baud
    rate (`docs/notes/mux-diagnostic-build.md`), so there is no `termios` configuration to
    get right here, only "don't block". `select.select` is used to wait for readability up
    to a caller-given timeout rather than polling in a busy loop. Line buffering and the
    partial-first-line discard are delegated to `_LineBuffer`.
    """

    def __init__(self, path: str):
        self._fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        self._lines = _LineBuffer()

    def close(self) -> None:
        os.close(self._fd)

    def readline(self, timeout_s: float) -> str | None:
        """Returns one decoded, newline-stripped line, or None if the timeout elapsed first."""
        deadline = time.monotonic() + timeout_s
        while True:
            line = self._lines.pop_line()
            if line is not None:
                return line

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            ready, _, _ = select.select([self._fd], [], [], remaining)
            if not ready:
                return None
            try:
                chunk = os.read(self._fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                # Device closed/disconnected: nothing more will ever arrive.
                return None
            self._lines.feed(chunk)


def _emit(report: DiagReport, as_json: bool) -> None:
    if as_json:
        print(json.dumps(report_to_dict(report)))
    else:
        print(format_summary(report))
    sys.stdout.flush()


def run_once(reader: _SerialLineReader, max_lines: int, timeout_s: float) -> DiagReport | None:
    """Reads raw serial lines, one at a time, up to `max_lines` (bounded overall by
    `timeout_s`), and returns as soon as the first complete, parseable verdict line is
    found. Non-decision chatter (the attach banner, the PWMREG line, a discarded partial
    first line) does not count toward finding a report, but each raw line read still counts
    against `max_lines`. Returns None if no verdict line was found before either bound hit.
    """
    deadline = time.monotonic() + timeout_s
    for _ in range(max_lines):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        line = reader.readline(remaining)
        if line is None:
            break
        parsed = parse_diag_line(line)
        if parsed is not None:
            return parsed
    return None


def run_watch(reader: _SerialLineReader, as_json: bool) -> int:
    """Prints a summary for every diagnostic line as it arrives, forever, until interrupted."""
    try:
        while True:
            line = reader.readline(timeout_s=3600.0)
            if line is None:
                continue
            parsed = parse_diag_line(line)
            if parsed is not None:
                _emit(parsed, as_json)
    except KeyboardInterrupt:
        return EXIT_PASS


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--device",
        default=DEFAULT_DEVICE,
        help=f"serial device to read (default: {DEFAULT_DEVICE})",
    )
    parser.add_argument(
        "--lines",
        type=int,
        default=DEFAULT_LINES,
        help=(
            "how many raw serial lines to read looking for a decision line before giving up "
            f"(default: {DEFAULT_LINES}); ignored with --watch"
        ),
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help=f"overall seconds to wait for data before giving up (default: {DEFAULT_TIMEOUT_S})",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="keep reading and printing one summary per line, like tail -f",
    )
    parser.add_argument("--json", action="store_true", help="print each report as a JSON object")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)

    try:
        reader = _SerialLineReader(args.device)
    except OSError as exc:
        print(f"read_mux_diag: cannot open {args.device}: {exc}", file=sys.stderr)
        return EXIT_NO_DATA

    try:
        if args.watch:
            return run_watch(reader, args.json)

        report = run_once(reader, args.lines, args.timeout)
        if report is None:
            print(
                f"read_mux_diag: timed out after {args.timeout}s (or {args.lines} raw "
                f"line(s)) with no parseable mux diagnostic line seen on {args.device} "
                "(wrong device, shipping firmware flashed instead of DIAG_BUILD, or "
                "nothing attached to USB? try a larger --lines/--timeout)",
                file=sys.stderr,
            )
            return EXIT_NO_DATA

        _emit(report, args.json)
        return exit_code_for(report)
    finally:
        reader.close()


if __name__ == "__main__":
    sys.exit(main())
