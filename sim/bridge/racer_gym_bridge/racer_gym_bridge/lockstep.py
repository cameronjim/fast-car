"""Simulated time and lockstep pacing for bridge_node (GitHub issue 156). Pure logic, no ROS.

With ``sim_time`` on, bridge_node owns the clock of the sim: it advances a counter by exactly one
env timestep per gym step, publishes it on /clock and stamps every message with it. Nodes run
with use_sim_time:=true, so what a node sees (which scan is paired with which odometry sample,
how far an estimate lags) depends on the stamps, not on how the OS happened to schedule a
starved 2-core runner.

Two limits keep sim time from outrunning the nodes under test:

  * WallPacer: sim time never advances faster than ``real_time_factor_max`` times wall time
    (default 1.0). The controllers (tracker_node, safety_node) tick on WALL timers by design
    (their CLOCK POLICY), so a sim that ran faster than real time would hand them fewer
    commands per sim second than on the car.
  * LagGate: sim time may not lead the stamped output of any acknowledging estimator (EKF,
    slam_toolbox, AMCL) by more than ``max_lag_s``. A starved estimator stops the clock until it
    has caught up, instead of the sim drifting ahead and the estimator dropping or mis-pairing
    messages. A gate that stays shut for ``timeout_s`` of wall time opens anyway and counts a
    timeout, so a dead node cannot hang a run; the count is logged.
"""

from __future__ import annotations

SIM_TIME_START_NS = 1_000_000_000  # never 0: stamp 0 means "latest" to tf2 and "unset" to nodes


class SimClock:
    """Exact integer simulated time: start + steps * timestep, no floating point drift."""

    def __init__(self, timestep_s: float, start_ns: int = SIM_TIME_START_NS) -> None:
        if timestep_s <= 0.0:
            raise ValueError(f"timestep_s must be positive, got {timestep_s}")
        self._step_ns = round(timestep_s * 1e9)
        self.now_ns = int(start_ns)

    def tick(self) -> int:
        self.now_ns += self._step_ns
        return self.now_ns


class WallPacer:
    """Releases steps no faster than ``rtf_max`` times real time, never in a burst.

    rtf_max <= 0 means unpaced. Deadlines accumulate (a poll that fires a little late does not
    slow the average rate), but a step more than one period late is not followed by a burst of
    catch-up steps: the deadline restarts from the moment it was taken."""

    def __init__(self, sim_dt_s: float, rtf_max: float) -> None:
        self._wall_dt_s = sim_dt_s / rtf_max if rtf_max > 0.0 else 0.0
        self._next_s: float | None = None

    @property
    def wall_period_s(self) -> float:
        return self._wall_dt_s

    def due(self, wall_s: float) -> bool:
        return self._next_s is None or wall_s >= self._next_s

    def taken(self, wall_s: float) -> None:
        if self._next_s is not None and wall_s - self._next_s < self._wall_dt_s:
            self._next_s += self._wall_dt_s
        else:
            self._next_s = wall_s + self._wall_dt_s


class LagGate:
    """Holds sim time back while an acknowledging estimator's output is too far behind it.

    Keys name estimator outputs (``ack_keys`` maps key -> lead_s, how far ahead of the data it
    used that output is stamped: 0 for an Odometry stamped with its data, 0.2 for AMCL's
    map -> odom, which is future-dated by its transform tolerance). A key is ARMED by its first
    message, so a node that is still starting never holds the sim back."""

    def __init__(self, ack_keys: dict[str, float], max_lag_s: float, timeout_s: float) -> None:
        if max_lag_s < 0.0 or timeout_s <= 0.0:
            raise ValueError("max_lag_s must be >= 0 and timeout_s > 0")
        self._lead_ns = {key: round(lead * 1e9) for key, lead in ack_keys.items()}
        self._latest_ns: dict[str, int] = {}
        self._max_lag_ns = round(max_lag_s * 1e9)
        self._timeout_s = timeout_s
        self._blocked_since_s: float | None = None
        self.timeouts = 0
        self.blocked_checks = 0

    @property
    def keys(self) -> list[str]:
        return list(self._lead_ns)

    def ack(self, key: str, stamp_ns: int) -> None:
        """Record an estimator output. Stamps never go backwards in the record."""
        if key in self._lead_ns and stamp_ns > self._latest_ns.get(key, -(2**62)):
            self._latest_ns[key] = int(stamp_ns)

    def behind(self, sim_now_ns: int) -> list[str]:
        """The armed keys whose output trails sim_now_ns by more than max_lag_s."""
        return [
            key
            for key, latest in self._latest_ns.items()
            if sim_now_ns - (latest - self._lead_ns[key]) > self._max_lag_ns
        ]

    def open(self, sim_now_ns: int, wall_s: float) -> bool:
        """May the sim step now? Counts a timeout when it has been held for timeout_s of wall."""
        if not self.behind(sim_now_ns):
            self._blocked_since_s = None
            return True
        self.blocked_checks += 1
        if self._blocked_since_s is None:
            self._blocked_since_s = wall_s
        if wall_s - self._blocked_since_s >= self._timeout_s:
            self.timeouts += 1
            self._blocked_since_s = None
            return True
        return False


def parse_ack_key(spec: str) -> tuple[str, float]:
    """``"/odom"`` -> ("/odom", 0.0); ``"tf:odom:0.2"`` -> ("tf:odom", 0.2).

    ``topic[:lead_s]`` for an Odometry topic; ``tf:<child_frame>[:lead_s]`` for a transform on
    /tf. The lead is the trailing ``:<number>`` when there is one."""
    head, sep, tail = spec.rpartition(":")
    if sep and head and head != "tf":
        try:
            return head, float(tail)
        except ValueError:
            pass
    return spec, 0.0
