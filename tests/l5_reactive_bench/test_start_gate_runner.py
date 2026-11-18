"""L1 tests (brief 51c item 3, fakes, no ROS): the runner starts the planner through the same
`StartReadyGate` the launch uses, the oracle's status is published in every phase and held back by
`--lap-truth-ready-after-s`, and the refusal ends the run with `start_refused`."""

from __future__ import annotations

import inspect
import math
import sys
from pathlib import Path

import pytest
import track_family_runner as runner

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros_ws/src/racer_laps/src"))
from racer_laps.start_ready import StartGateConfig, StartReadyGate  # noqa: E402


def stack():
    return [
        (name, [name])
        for name in (
            "bridge_node",
            "safety_node",
            "track_perception_node",
            "lap_memory_node",
            "mppi_planner_node",
        )
    ]


def ready_status(**changes):
    out = {
        "lap_source": "truth",
        "phase": "waiting_for_driving",
        "source_ready_now": True,
        "source_alive": True,
        "source_not_ready_reason": "",
    }
    out.update(changes)
    return out


class Clock:
    """A fake node clock that advances by one status period per spin, feeding the gate the way the
    runner's subscription does."""

    def __init__(self, statuses_from_poll: int, bad: bool = False, period_s: float = 1.0) -> None:
        self.t = 100.0
        self.polls = 0
        self.first = statuses_from_poll
        self.bad = bad
        self.period = period_s
        self.gate = StartReadyGate(StartGateConfig(), self.t)

    def spin(self) -> None:
        self.polls += 1
        self.t += self.period
        if self.polls >= self.first:
            body = ready_status(source_ready_now=False) if self.bad else ready_status()
            if self.bad:
                body["source_not_ready_reason"] = "source state 'waiting', not 'running'"
            self.gate.on_status(self.t, body)

    def decide(self):
        return self.gate.decide(self.t)


def start(clock: Clock, log: list[str], polls: int = 200):
    return runner.run_startup(
        stack(),
        two_phase=True,
        spawn=lambda name, _c: log.append(f"spawn {name}"),
        ready=lambda: True,
        spin=clock.spin,
        reset=lambda: log.append("reset"),
        after_reset=lambda: log.append("clear"),
        wait_timeout_polls=polls,
        start_decide=clock.decide,
    )


def test_the_planner_is_spawned_only_after_the_gate_says_start() -> None:
    clock, log = Clock(statuses_from_poll=3), []
    verdict = start(clock, log)
    assert verdict == ("start", "")
    assert log[-1] == "spawn mppi_planner_node"  # the last node
    assert log.index("spawn lap_memory_node") < log.index("spawn mppi_planner_node")
    # two ready statuses were needed: statuses from poll 3 on, the second at poll 4
    assert clock.polls == 4
    assert clock.gate.trace()["verdict"] == "start"


def test_on_refuse_the_planner_is_never_spawned_and_the_verdict_names_the_reason() -> None:
    clock, log = Clock(statuses_from_poll=1, bad=True), []
    verdict = start(clock, log)
    assert verdict == ("refuse", "source state 'waiting', not 'running'")
    assert "spawn mppi_planner_node" not in log
    assert "spawn lap_memory_node" in log  # everything before the planner did start
    assert clock.t - 100.0 > 30.0  # it waited out wait_s of the node's clock


def test_no_status_at_all_refuses_with_no_lap_node_status() -> None:
    clock, log = Clock(statuses_from_poll=10**9), []
    assert start(clock, log) == ("refuse", "no lap node status")
    assert "spawn mppi_planner_node" not in log


def test_a_gate_that_cannot_decide_in_the_polls_is_an_error_and_the_planner_never_starts() -> None:
    clock, log = Clock(statuses_from_poll=10**9, period_s=0.01), []
    with pytest.raises(RuntimeError, match="lap_memory_node did not answer"):
        start(clock, log, polls=20)
    assert "spawn mppi_planner_node" not in log


def test_with_another_lap_source_the_order_is_todays() -> None:
    log: list[str] = []
    answered = {"n": 0}

    def lap_ready() -> bool:
        answered["n"] += 1
        return answered["n"] > 3

    verdict = runner.run_startup(
        stack(),
        two_phase=True,
        spawn=lambda name, _c: log.append(f"spawn {name}"),
        ready=lambda: True,
        spin=lambda: log.append("spin"),
        reset=lambda: log.append("reset"),
        after_reset=lambda: log.append("clear"),
        wait_timeout_polls=100,
        lap_ready=lap_ready,
    )
    assert verdict == ("start", "")
    assert answered["n"] == 4 and log[-1] == "spawn mppi_planner_node"


def test_without_a_lap_node_nothing_waits_even_with_a_gate() -> None:
    plain = [item for item in stack() if item[0] != "lap_memory_node"]
    out: list[str] = []
    verdict = runner.run_startup(
        plain,
        two_phase=True,
        spawn=lambda name, _c: out.append(name),
        ready=lambda: True,
        spin=lambda: None,
        reset=lambda: None,
        after_reset=lambda: None,
        wait_timeout_polls=3,
        start_decide=lambda: ("wait", ""),
    )
    assert verdict == ("start", "") and out[-1] == "mppi_planner_node"


# -- the oracle's status --------------------------------------------------------------------------


def oracle_status() -> dict:
    return {"state": "running", "similarity": 1.0, "laps": 0, "armed": False}


@pytest.mark.parametrize(
    ("ready_after", "lap_first", "now", "ready"),
    [
        (0.0, None, 5.0, True),  # the default: as before, ready whatever the lap node did
        (0.0, 10.0, 10.0, True),
        (6.0, None, 50.0, False),  # the lap node has not spoken: still waiting
        (6.0, 10.0, 15.999, False),
        (6.0, 10.0, 16.0, True),  # exactly the delay after the lap node's first status
        (6.0, 10.0, 99.0, True),
        (math.inf, 10.0, 1e9, False),  # never
        (math.inf, None, 1.0, False),
    ],
)
def test_the_oracle_status_is_held_until_the_delay_after_the_lap_nodes_first_status(
    ready_after: float, lap_first: float | None, now: float, ready: bool
) -> None:
    out = runner.oracle_status_at(oracle_status(), now, lap_first, ready_after)
    if ready:
        assert out == oracle_status()
    else:
        assert out["state"] == "waiting" and out["similarity"] == 0.0
        assert out["laps"] == 0  # the rest of the status is the oracle's


def test_the_status_publisher_speaks_once_a_second_from_the_first_sample_in_every_phase() -> None:
    published: list[tuple[float, dict]] = []
    pub = runner.OracleStatusPublisher(
        oracle_status, lambda body: published.append(body), ready_after_s=0.0
    )
    t = 0.0
    for _ in range(1000):  # 20 s of 50 Hz ground-truth samples, no `drive_raw`, no planner
        pub.on_time(t, now_s=t, lap_first_status_s=None)
        t += 0.02
    assert len(published) == 20
    assert pub.first_publish_s == 0.0


def test_the_publisher_holds_the_status_and_still_publishes_it() -> None:
    published: list[dict] = []
    pub = runner.OracleStatusPublisher(oracle_status, published.append, ready_after_s=6.0)
    for k in range(20):
        t = float(k)
        pub.on_time(t, now_s=t, lap_first_status_s=3.0 if t >= 3.0 else None)
    states = [p["state"] for p in published]
    assert states[:9] == ["waiting"] * 9  # t = 0 .. 8: before 3.0 + 6.0
    assert states[9:] == ["running"] * 11
    assert len(published) == 20


def test_the_publisher_is_fed_from_the_ground_truth_odometry_callback_not_the_main_loop() -> None:
    """Brief 51 section 4 said so and the build put it in the main loop's pass over the recording,
    which begins after the planner's first /drive_raw (brief 51c)."""
    source = inspect.getsource(runner.run_benchmark)
    on_odom = source[source.index("def on_odom(msg)") : source.index("def received_s()")]
    assert "oracle_status" in on_odom and "on_time(" in on_odom
    main_pass = source[source.index("while consumed < len(recording.odom)") :]
    assert "oracle_status.on_time" not in main_pass.split("previous = sample")[0]


def test_the_result_says_start_refused_and_carries_the_refusal() -> None:
    out = runner.refusal_result_keys(
        ("refuse", "source state 'waiting', not 'running'"),
        {"verdict": "refuse", "refuse_s": 132.0},
        refused_at_s=132.0,
        drive_raw_messages=0,
    )
    assert out["stopped_because"] == "start_refused"
    assert out["start_refusal"] == {
        "reason": "source state 'waiting', not 'running'",
        "refused_at_s": 132.0,
        "gate": {"verdict": "refuse", "refuse_s": 132.0},
    }
    assert out["drive_raw_messages"] == 0
