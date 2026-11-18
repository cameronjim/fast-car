"""lap_source_ready_wait: the car's start gate for the camera lap source (brief 51c).

`pnp.launch.py` runs this node after the camera lap counter is in the graph and its reset call has
returned, and starts the planner (the only publisher of `/drive_raw`) only when it exits 0. It
subscribes to the lap node's `<prefix>/status` (reliable, depth 10: a best-effort publisher is not
matched, and the wait then ends in "no lap node status"), feeds `racer_laps.start_ready.StartReadyGate`
on the wall clock, logs one line per change of the reason it is still waiting, and exits

* 0 when the gate says `start` (two consecutive ready statuses, `source_ready_now` and
  `source_alive`, at most 2.5 s apart);
* 3 when it says `refuse` (30 s without that, or a lap node that is already running), the reason
  being the last log line (`LAST_LINE_PREFIX` and the lap node's own `source_not_ready_reason`).

No publisher, no service, and no parameter that shortens the wait or skips it: the constants are
the pre-registered `start_gate` block (`StartGateConfig`), and only a test passes another config to
`main`. The core, `ReadyWaiter`, is plain Python and tested without ROS.
"""

from __future__ import annotations

import json
import logging
import sys
import time
from collections.abc import Callable
from typing import Any

from racer_laps.start_ready import (
    VERDICT_REFUSE,
    VERDICT_START,
    StartGateConfig,
    StartReadyGate,
)

EXIT_START = 0
EXIT_REFUSE = 3
#: The last log line of a refusal is this followed by the reason. The launch reads it back.
LAST_LINE_PREFIX = "lap source not ready: "
WAITING_PREFIX = "waiting for the lap node: "
POLL_PERIOD_S = 0.1


def reason_of_last_line(line: str) -> str:
    """The reason of a refusal from a log line that carries `LAST_LINE_PREFIX` (anything before it
    on the line, such as a logger's name, is dropped); a line without it is returned as it is."""
    index = line.rfind(LAST_LINE_PREFIX)
    if index < 0:
        return line.strip()
    return line[index + len(LAST_LINE_PREFIX) :].strip()


class ReadyWaiter:
    """`StartReadyGate` behind JSON messages and a log. `step(t)` returns None while waiting,
    else the exit code."""

    def __init__(self, cfg: StartGateConfig, t0_s: float, log: Callable[[str], None]) -> None:
        self.gate = StartReadyGate(cfg, t0_s)
        self._log = log
        self._last_reason: str | None = None
        self._last_status: Any = None
        self._code: int | None = None
        self.malformed = 0

    def on_message(self, t_s: float, data: str) -> None:
        """One `<prefix>/status` message (its JSON text) that arrived at `t_s`."""
        try:
            body = json.loads(data)
        except (ValueError, TypeError):
            self.malformed += 1
            body = None
        if body is not None and not isinstance(body, dict):
            self.malformed += 1
            body = None
        self._last_status = body
        self.gate.on_status(t_s, body if body is not None else {})

    def _waiting_reason(self) -> str:
        status = self._last_status
        if status is None:
            return (
                "no lap node status" if self.gate.trace()["statuses"] == 0 else "status unreadable"
            )
        reason = status.get("source_not_ready_reason")
        if isinstance(reason, str) and reason:
            return reason
        return "the lap node's status does not say ready yet"

    def step(self, t_s: float) -> int | None:
        if self._code is not None:
            return self._code
        verdict, reason = self.gate.decide(t_s)
        if verdict == VERDICT_START:
            self._log("lap source ready on two consecutive statuses: starting the planner")
            self._code = EXIT_START
        elif verdict == VERDICT_REFUSE:
            self._log(f"{LAST_LINE_PREFIX}{reason}")
            self._code = EXIT_REFUSE
        else:
            now = self._waiting_reason()
            if now != self._last_reason:
                self._last_reason = now
                self._log(f"{WAITING_PREFIX}{now}")
        return self._code


def main(
    argv: list[str] | None = None, gate_config: StartGateConfig | None = None
) -> int:  # pragma: no cover
    """The node. `gate_config` is for tests only (a shorter `wait_s`); the launch never passes
    one and the node has no parameter for it."""
    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=argv)
    node = rclpy.create_node("lap_source_ready_wait")
    node.declare_parameter("status_topic", "/pnp/lap/status")
    topic = str(node.get_parameter("status_topic").value)
    logger = node.get_logger()
    started = time.monotonic()
    waiter = ReadyWaiter(gate_config or StartGateConfig(), 0.0, logger.info)

    def now() -> float:
        return time.monotonic() - started

    def on_status(msg: String) -> None:
        waiter.on_message(now(), msg.data)

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )
    node.create_subscription(String, topic, on_status, qos)
    logger.info(f"waiting for {topic}: the planner starts when the lap source is ready")
    code: int | None = None
    try:
        while rclpy.ok() and code is None:
            rclpy.spin_once(node, timeout_sec=POLL_PERIOD_S)
            code = waiter.step(now())
        if code is None:  # interrupted: not ready, never start
            code = EXIT_REFUSE
            logger.info(f"{LAST_LINE_PREFIX}interrupted before the lap source was ready")
    except KeyboardInterrupt:
        code = EXIT_REFUSE
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":  # pragma: no cover
    logging.basicConfig(level=logging.INFO)
    sys.exit(main())
