"""tools/pnp_session/lap_src.py and streamread.py (brief 51): D9-C0 items (i) to (v) and D9-B0 items
(i) to (iii) at pass, at the threshold and epsilon over, the wheel rule standing in for a missing
count, under 10 laps, a bag with no camera-source lap node, the shadow topics, the freeze guard and
the registered hash, a bag written with mcap-ros2-support, a truncated copy of it and the command
line. The synthetic runs are a car at 1 m/s on a loop of 5.2 m, a camera that counts every lap."""

from __future__ import annotations

import io
import json
import math
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

pytest.importorskip("mcap")
pytest.importorskip("racer_eval")
pytest.importorskip("mcap_ros2")

from pnp_session import cli, lap_src, streamread
from pnp_session.lap_src import FAIL, INSUFFICIENT, PASS, REPEAT

TH = lap_src.load_prereg()
LAP = 5.2
SPEED = 1.0
T0_NS = 1_700_000_000 * 1_000_000_000
T_DRIVE = 2.0  # the car starts moving; the planner has been DRIVING since 1.0
WHEELBASE = 0.33
EPS = 1e-6
TWO_PI = 2.0 * math.pi


def _stamp(t: float) -> SimpleNamespace:
    return SimpleNamespace(sec=int(t), nanosec=round((t - int(t)) * 1e9))


def ns(t: float) -> int:
    return T0_NS + round(t * 1e9)


@dataclass
class Scenario:
    """The messages of a run, on the bag axis (seconds after the first /scan)."""

    n: int = 12
    lap_m: float = LAP  # the period of the steering (the track)
    event_lap_m: float = LAP  # driven between two camera events
    peaks: dict[int, float] = field(default_factory=dict)  # lap -> peak similarity
    distance_delta: dict[int, float] = field(default_factory=dict)  # lap -> added to distance_m
    lap_time_delta: dict[int, float] = field(default_factory=dict)
    latency: dict[int, float] = field(default_factory=dict)  # lap -> arrival minus crossing
    node_length_delta: dict[int, float] = field(default_factory=dict)
    status_gap: tuple[float, float] | None = None  # no camera status in this window
    status_after: tuple[float, float] | None = None  # (time, extra gap): a status gap of 1 + extra
    node_prefix: str | None = "/pnp/lap_shadow"
    node_source: str = "camera"
    node_ready: bool = True
    node_confirmed: int | None = None
    node_doubtful: list[dict[str, Any]] = field(default_factory=list)
    node_rejected: int = 0
    node_suspect_lap: int | None = None
    odometry: bool = True
    first_status_state: str = "running"
    steering_steps: bool = True

    def crossing(self, k: int) -> float:
        return T_DRIVE + k * self.event_lap_m / SPEED

    def messages(self) -> list[tuple[int, str, Any]]:
        msgs: list[tuple[int, str, Any]] = []

        def add(t: float, topic: str, msg: Any) -> None:
            msgs.append((ns(t), topic, msg))

        end = self.crossing(self.n) + 2.0
        k = 0
        while k * 0.02 <= end:
            t = k * 0.02
            v = SPEED if t >= T_DRIVE else 0.0
            if self.odometry:
                add(
                    t,
                    "/odom/wheel",
                    SimpleNamespace(
                        header=SimpleNamespace(stamp=_stamp(T0_NS / 1e9 + t)),
                        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=v))),
                    ),
                )
            s = max(0.0, (t - T_DRIVE) * SPEED)
            tan = TWO_PI * WHEELBASE / self.lap_m
            if self.steering_steps:
                tan += 0.15 * math.sin(TWO_PI * s / self.lap_m)  # a periodic steering signal
            steer = math.atan(tan) if v > 0 else 0.0
            add(t, "/drive", SimpleNamespace(drive=SimpleNamespace(steering_angle=steer, speed=v)))
            add(
                t,
                "/drive_raw",
                SimpleNamespace(drive=SimpleNamespace(steering_angle=steer, speed=v)),
            )
            add(
                t,
                "/pnp/planner/status",
                SimpleNamespace(state=1 if t >= 1.0 else 0, plan_min_clearance_m=0.3),
            )
            k += 1
        # camera status at 1 Hz, the reference taken standing
        status_times: list[float] = []
        t = 0.5
        while t <= end:
            status_times.append(t)
            gap = 1.0
            if self.status_after and abs(t - self.status_after[0]) < 1e-9:
                gap += self.status_after[1]
            t += gap
        laps_seen = 0
        for t in status_times:
            while laps_seen < self.n and self.crossing(laps_seen + 1) + 0.16 <= t:
                laps_seen += 1
            if not (self.status_gap and self.status_gap[0] <= t <= self.status_gap[1]):
                body = {
                    "state": self.first_status_state if t < 3.0 else "running",
                    "similarity": 1.0 if laps_seen == 0 and t < T_DRIVE else 0.4,
                    "laps": laps_seen,
                }
                add(t, "/lap/status", SimpleNamespace(data=json.dumps(body)))
        t0_epoch = T0_NS / 1e9
        for k in range(1, self.n + 1):
            c = self.crossing(k)
            body = {
                "lap": k,
                "lap_time_s": self.event_lap_m / SPEED if k > 1 else c,
                "peak_similarity": self.peaks.get(k, 0.93),
                "crossing_time_s": t0_epoch + c,
                "distance_m": self.event_lap_m + self.distance_delta.get(k, 0.0),
            }
            if k in self.lap_time_delta:
                body["lap_time_s"] += self.lap_time_delta[k]
            arrival = c + self.latency.get(k, 0.16)
            add(arrival, "/lap/event", SimpleNamespace(data=json.dumps(body)))
            if self.node_prefix:
                add(
                    arrival + 0.01,
                    f"{self.node_prefix}/event",
                    SimpleNamespace(
                        header=SimpleNamespace(stamp=_stamp(t0_epoch + c)),
                        lap_index=k,
                        lap_time_s=self.event_lap_m / SPEED,
                        lap_length_m=self.event_lap_m + self.node_length_delta.get(k, 0.0),
                        v_cap_mps=0.5,
                        clean=True,
                    ),
                )
        if self.node_prefix:
            confirmed = self.n if self.node_confirmed is None else self.node_confirmed
            for t in (end - 2.0, end):
                status = {
                    "lap_source": self.node_source,
                    "lap_mode": "observe",
                    "source_ready": self.node_ready,
                    "source_not_ready_reason": "" if self.node_ready else "similarity 0.08",
                    "confirmed_laps": confirmed,
                    "doubtful_laps": self.node_doubtful,
                    "rejected_events": {"latency": self.node_rejected, "malformed": 0},
                    "reference_suspect": self.node_suspect_lap is not None,
                    "reference_suspect_lap": self.node_suspect_lap,
                    "missed_gate_laps": [d["lap"] for d in self.node_doubtful],
                }
                add(t, f"{self.node_prefix}/status", SimpleNamespace(data=json.dumps(status)))
        msgs.sort(key=lambda m: m[0])
        return msgs

    def bag(self, name: str = "synthetic") -> lap_src.BagData:
        return lap_src.bag_data_from_messages(name, T0_NS, self.messages())


def c0(scenario: Scenario, count: int | None = None) -> dict[str, Any]:
    return lap_src.evaluate_c0(scenario.bag(), count, TH)


def item(block: dict[str, Any], key: str) -> dict[str, Any]:
    return next(i for i in block["items"] if i["item"].startswith(f"({key})"))


# -- the constants -------------------------------------------------------------------------------


def test_the_prereg_is_read_through_its_hash(tmp_path: Path) -> None:
    assert TH["d9_c0"]["peak_similarity_min"] == 0.88
    assert TH["d9_b0"]["length_tolerance_m"] == 0.10
    bad = tmp_path / "x.yaml"
    bad.write_text("schema: nope\n")
    with pytest.raises(lap_src.PreregError):
        lap_src.load_prereg(bad)


def test_the_hash_constant_is_the_files() -> None:
    import hashlib

    from pnp_session.constants import D9_YAML, D9_YAML_SHA256

    path = Path(__file__).resolve().parents[2] / D9_YAML
    assert hashlib.sha256(path.read_bytes()).hexdigest() == D9_YAML_SHA256


# -- D9-C0 ------------------------------------------------------------------------------------------


def test_a_clean_run_passes_every_item_with_the_owners_count() -> None:
    block = c0(Scenario(), count=12)
    assert block["verdict"] == PASS, block["items"]
    assert [i["verdict"] for i in block["items"]] == [PASS] * 5
    assert block["lap_node"] == "/pnp/lap_shadow" and block["camera_laps"] == 12


def test_item_i_the_count_equals_the_owners() -> None:
    assert item(c0(Scenario(), 12), "i")["verdict"] == PASS
    assert item(c0(Scenario(), 11), "i")["verdict"] == FAIL
    assert item(c0(Scenario(), 13), "i")["verdict"] == FAIL


@pytest.mark.parametrize(("delta", "verdict"), [(0.52, PASS), (0.52 + EPS, FAIL), (-0.52, PASS)])
def test_item_i_every_lap_within_ten_percent_of_the_median(delta: float, verdict: str) -> None:
    block = c0(Scenario(distance_delta={1: delta}), 12)
    assert item(block, "i")["verdict"] == verdict


def test_item_i_the_median_is_over_laps_2_to_10() -> None:
    sc = Scenario(distance_delta={1: 0.0, 11: 3.0, 12: 3.0})  # laps 11 and 12 are not judged
    assert item(c0(sc, 12), "i")["verdict"] == PASS


@pytest.mark.parametrize(
    ("peak", "verdict"), [(0.88, PASS), (0.88 - EPS, FAIL), (0.95, PASS), (0.80, FAIL)]
)
def test_item_ii_every_peak_at_least_0_88(peak: float, verdict: str) -> None:
    assert item(c0(Scenario(peaks={7: peak}), 12), "ii")["verdict"] == verdict


def test_item_ii_only_the_first_ten_laps_count() -> None:
    assert item(c0(Scenario(peaks={11: 0.5, 12: 0.5}), 12), "ii")["verdict"] == PASS


def test_item_iii_the_node_must_have_started_ready_and_confirmed_ten_laps() -> None:
    assert item(c0(Scenario(node_ready=False), 12), "iii")["verdict"] == FAIL
    assert item(c0(Scenario(node_confirmed=9), 12), "iii")["verdict"] == FAIL
    assert item(c0(Scenario(node_confirmed=10), 12), "iii")["verdict"] == PASS
    doubtful = [{"lap": 4, "event_lap": 4, "reasons": ["length"]}]
    assert item(c0(Scenario(node_doubtful=doubtful), 12), "iii")["verdict"] == FAIL
    late = [{"lap": 11, "event_lap": 11, "reasons": ["length"]}]  # after the judged laps
    assert item(c0(Scenario(node_doubtful=late, node_confirmed=11), 12), "iii")["verdict"] == PASS
    assert item(c0(Scenario(node_rejected=1), 12), "iii")["verdict"] == FAIL


@pytest.mark.parametrize(("delta", "verdict"), [(0.10, PASS), (0.10 + EPS, FAIL), (-0.10, PASS)])
def test_item_iii_lap_lengths_within_0_10_m_of_the_events(delta: float, verdict: str) -> None:
    block = c0(Scenario(node_length_delta={5: delta}), 12)
    assert item(block, "iii")["verdict"] == verdict


def test_item_iii_lap_1_is_not_compared_and_a_missing_node_event_fails() -> None:
    assert item(c0(Scenario(node_length_delta={1: 1.0}), 12), "iii")["verdict"] == PASS
    sc = Scenario()
    bag = sc.bag()
    node = bag.nodes["/pnp/lap_shadow"]
    node.events = [e for e in node.events if e["lap_index"] != 4]
    out = lap_src.evaluate_c0(bag, 12, TH)
    assert item(out, "iii")["verdict"] == FAIL
    assert item(out, "iii")["numbers"]["laps_with_no_node_event"] == [4]


def test_item_iii_without_source_keys_is_insufficient() -> None:
    bag = Scenario().bag()
    st = bag.nodes["/pnp/lap_shadow"].statuses[-1][1]
    del st["source_ready"]
    assert item(lap_src.evaluate_c0(bag, 12, TH), "iii")["verdict"] == INSUFFICIENT


@pytest.mark.parametrize(("latency", "verdict"), [(0.5, PASS), (0.5 + EPS, FAIL), (0.09, PASS)])
def test_item_iv_no_event_later_than_0_5_s(latency: float, verdict: str) -> None:
    assert item(c0(Scenario(latency={6: latency}), 12), "iv")["verdict"] == verdict


@pytest.mark.parametrize(
    ("extra", "verdict"), [(0.0, PASS), (1.0, PASS), (2.0, PASS), (2.0 + EPS, FAIL), (5.0, FAIL)]
)
def test_item_v_no_status_gap_above_3_s(extra: float, verdict: str) -> None:
    """A status gap of 1 + extra seconds after the status at 20.5 s (1 Hz otherwise)."""
    out = item(c0(Scenario(status_after=(20.5, extra)), 12), "v")
    assert out["verdict"] == verdict, out
    assert out["numbers"]["largest_gap_s"] == pytest.approx(max(1.0, 1.0 + extra))


def test_item_v_without_any_status_is_insufficient() -> None:
    bag = Scenario().bag()
    bag.cam_status = []
    assert item(lap_src.evaluate_c0(bag, 12, TH), "v")["verdict"] == INSUFFICIENT


def test_the_wheel_rule_stands_in_for_a_missing_count() -> None:
    block = c0(Scenario(), None)
    i = item(block, "i")
    assert i["verdict"] == PASS and "wheel rule" in i["numbers"]["expected_from"]
    assert "no count was taken" in i["note"]
    # a camera that missed laps: 12 events for 14 laps of wheel distance
    sc = Scenario(n=12)
    bag = sc.bag()
    s = bag.streams
    bag.wheel_distance_m = 14.2 * LAP
    out = lap_src.evaluate_c0(bag, None, TH)
    assert item(out, "i")["verdict"] == FAIL and out["verdict"] == FAIL
    assert s is bag.streams
    # no odometry at all: the rule cannot be applied, repeat with the count taken
    out = lap_src.evaluate_c0(Scenario(odometry=False).bag(), None, TH)
    assert item(out, "i")["verdict"] == REPEAT and out["verdict"] == REPEAT


def test_a_wheel_count_with_no_distances_cannot_be_applied() -> None:
    bag = Scenario().bag()
    for e in bag.cam_events:
        e["distance_m"] = math.nan
    out = lap_src.evaluate_c0(bag, None, TH)
    assert item(out, "i")["verdict"] == REPEAT
    bag = Scenario().bag()
    count, why = lap_src._wheel_count([0.0, 0.0], 50.0)
    assert count is None and "median" in why


def test_under_ten_camera_laps_is_insufficient() -> None:
    block = c0(Scenario(n=9), 9)
    assert block["verdict"] == INSUFFICIENT
    assert [i["verdict"] for i in block["items"]] == [INSUFFICIENT] * 5
    assert c0(Scenario(n=10), 10)["verdict"] == PASS


def test_a_bag_with_no_camera_source_lap_node_is_refused_clearly() -> None:
    with pytest.raises(lap_src.NoCameraNode, match="no lap node on the camera source"):
        lap_src.evaluate_c0(Scenario(node_prefix=None).bag(), 12, TH)
    with pytest.raises(lap_src.NoCameraNode, match="start_scan"):
        lap_src.evaluate_c0(Scenario(node_source="start_scan").bag(), 12, TH)


def test_the_primary_and_the_shadow_topics_are_both_found() -> None:
    assert c0(Scenario(node_prefix="/pnp/lap"), 12)["lap_node"] == "/pnp/lap"
    assert c0(Scenario(node_prefix="/pnp/lap_shadow"), 12)["lap_node"] == "/pnp/lap_shadow"
    # both present: the primary when it is on the camera source, else the shadow
    a, b = Scenario(node_prefix="/pnp/lap", node_source="start_scan"), Scenario()
    bag = a.bag()
    shadow = b.bag().nodes["/pnp/lap_shadow"]
    bag.nodes["/pnp/lap_shadow"] = shadow
    assert lap_src.camera_node(bag).prefix == "/pnp/lap_shadow"
    bag2 = Scenario(node_prefix="/pnp/lap").bag()
    bag2.nodes["/pnp/lap_shadow"] = shadow
    assert lap_src.camera_node(bag2).prefix == "/pnp/lap"


# -- the drive table ----------------------------------------------------------------------------------


def test_the_drive_table_shows_verdicts_reasons_caps_and_clean_flags() -> None:
    doubtful = [{"lap": 3, "event_lap": 3, "reasons": ["length", "heading"]}]
    bag = Scenario(node_prefix="/pnp/lap", node_doubtful=doubtful, node_confirmed=11).bag()
    table = lap_src.drive_table(bag)
    assert table["lap_node"] == "/pnp/lap" and len(table["rows"]) == 12
    row3 = table["rows"][2]
    assert row3["verdict"] == "doubtful" and row3["reasons"] == ["length", "heading"]
    assert table["rows"][0]["verdict"] == "confirmed" and table["rows"][0]["clean"] is True
    text = lap_src.markdown_drive(table)
    assert "| 3 | doubtful | length,heading |" in text
    with pytest.raises(lap_src.NoCameraNode):
        lap_src.drive_table(Scenario(node_prefix=None).bag())


# -- D9-B0 ----------------------------------------------------------------------------------------------

ODOM = None


def odom_model() -> Any:
    lap_src.racer_laps_path()
    from racer_laps.gate import GateConfig

    return GateConfig(wheelbase_m=WHEELBASE, yaw_gain_table=((0.0, 1.0),))


def b0(scenario: Scenario) -> dict[str, Any]:
    runs = lap_src.replay_runs(scenario.bag(), odom_model(), TH)
    return lap_src.evaluate_b0(runs, TH)


def b0_item(block: dict[str, Any], key: str) -> dict[str, Any]:
    return next(i for i in block["items"] if i["item"].startswith(f"({key})"))


def test_b0_a_clean_run_passes_every_item() -> None:
    block = b0(Scenario())
    assert block["verdict"] == PASS, block
    assert block["judged_laps"] == 11  # every camera event after the first of the run
    assert [i["verdict"] for i in block["items"]] == [PASS] * 3
    run = block["runs"][0]
    assert run["source_ready_real"] is True and run["lapref_laps"] > 0
    assert run["real_ready"]["confirmed"] == run["real_ready"]["laps"] == 12
    assert block["laps"][0]["judged"] is False and block["laps"][1]["judged"] is True


@pytest.mark.parametrize(("delta", "verdict"), [(0.0199, PASS), (0.0201, FAIL), (-0.0199, PASS)])
def test_b0_item_i_lap_time_within_0_02_s(delta: float, verdict: str) -> None:
    block = b0(Scenario(lap_time_delta={5: delta}))
    assert b0_item(block, "i")["verdict"] == verdict


def test_b0_item_i_a_rejected_event_is_a_failure_of_time_and_length() -> None:
    block = b0(Scenario(latency={5: 3.0}))  # arrives 3 s after its crossing: rejected
    assert b0_item(block, "i")["verdict"] == FAIL and b0_item(block, "ii")["verdict"] == FAIL
    assert block["verdict"] == FAIL


@pytest.mark.parametrize(("delta", "verdict"), [(0.10, PASS), (0.1001, FAIL), (-0.1001, FAIL)])
def test_b0_item_ii_length_within_0_10_m(delta: float, verdict: str) -> None:
    block = b0(Scenario(distance_delta={6: delta}))
    assert b0_item(block, "ii")["verdict"] == verdict


def test_b0_item_iii_a_camera_that_counts_every_1_17_laps_is_a_failure() -> None:
    """Every event 17 percent of a lap late: all laps confirmed (equal lengths, heading within the
    window) but outside 0.85 to 1.15 of the lapref length: confirmed outside the band."""
    sc = Scenario(lap_m=LAP, event_lap_m=LAP * 1.17, n=12)
    block = b0(sc)
    iii = b0_item(block, "iii")
    assert iii["verdict"] == FAIL and iii["numbers"]["confirmed_outside_band"] >= 1
    assert block["verdict"] == FAIL


def test_b0_item_iii_inside_the_band_at_least_ninety_percent_must_be_confirmed() -> None:
    # the camera status falls silent for 20 s: the laps of that stretch are doubtful (alive)
    sc = Scenario(n=12, status_gap=(15.0, 40.0))
    block = b0(sc)
    iii = b0_item(block, "iii")
    assert iii["verdict"] == FAIL and iii["numbers"]["inside_band_share"] < 0.90
    # one doubtful lap in eleven is under ten percent: 10 of 11 inside the band is 0.909
    sc = Scenario(n=12, status_after=(30.5, 2.5))  # one lap sees a 3.5 s gap
    assert b0_item(b0(sc), "iii")["numbers"]["inside_band_share"] >= 0.90


def test_b0_under_ten_judged_laps_is_insufficient_not_fail() -> None:
    block = b0(Scenario(n=8))
    assert block["verdict"] == INSUFFICIENT and "fewer than 10" in block["insufficient_because"]
    block = b0(Scenario(n=11))  # ten judged laps: enough
    assert block["verdict"] == PASS and block["judged_laps"] == 10


def test_b0_no_run_with_a_lapref_period_is_insufficient() -> None:
    block = b0(Scenario(steering_steps=False))  # a constant steering has no period
    assert block["verdict"] == INSUFFICIENT
    assert b0_item(block, "iii")["verdict"] == INSUFFICIENT
    assert "lapref" in block["insufficient_because"]


def test_b0_a_not_ready_run_reports_the_real_rule_and_judges_with_ready_assumed() -> None:
    sc = Scenario(first_status_state="waiting")  # the camera still says waiting when the car starts
    block = b0(sc)
    run = block["runs"][0]
    assert run["source_ready_real"] is False and "waiting" in run["source_not_ready_reason"]
    assert run["real_ready"]["confirmed"] == 0 and run["real_ready"]["doubtful"] == 12
    assert run["assumed_ready"]["confirmed"] == 12
    assert block["verdict"] == PASS  # item iii judges with ready assumed


def test_b0_markdown_has_both_replays_and_the_label() -> None:
    block = b0(Scenario(first_status_state="waiting"))
    text = lap_src.markdown_b0(block, "REGRESSION, NOT A GATE: x", assume_ready=False)
    assert "REGRESSION, NOT A GATE" in text and "real ready rule: source_ready False" in text
    assumed = lap_src.markdown_b0(block, "x", assume_ready=True)
    assert "confirmed" in assumed and "(lap 1, not judged)" in assumed


# -- the freeze guard --------------------------------------------------------------------------------------


def test_an_evaluation_bag_is_refused_before_it_is_opened_unless_the_freeze_exists(
    tmp_path: Path,
) -> None:
    names = lap_src.evaluation_bags(TH)
    assert names == ["2026-11-16T22-45-39_car_teleop", "2026-11-16T23-04-09_car_teleop"]
    freeze = tmp_path / "b9.json"
    for name in names:
        with pytest.raises(lap_src.FreezeRequired, match="not opened before the freeze"):
            lap_src.guard_bags([tmp_path / name], TH, freeze)
    # a development bag never needs it
    assert lap_src.guard_bags([tmp_path / "2026-11-16T22-38-47_car_teleop"], TH, freeze) is False
    freeze.write_text(json.dumps({"commit": "0" * 40, "yaml_sha256": "wrong"}))
    with pytest.raises(lap_src.FreezeRequired):
        lap_src.guard_bags([tmp_path / names[0]], TH, freeze)
    freeze.write_text("{")
    with pytest.raises(lap_src.FreezeRequired):
        lap_src.guard_bags([tmp_path / names[0]], TH, freeze)
    freeze.write_text(json.dumps({"commit": "xyz", "yaml_sha256": lap_src.D9_YAML_SHA256}))
    with pytest.raises(lap_src.FreezeRequired):
        lap_src.guard_bags([tmp_path / names[0]], TH, freeze)
    freeze.write_text("[]")
    assert lap_src.freeze_state(freeze)[0] is False
    freeze.write_text(json.dumps({"commit": "a" * 40, "yaml_sha256": lap_src.D9_YAML_SHA256}))
    assert lap_src.freeze_state(freeze) == (True, "")
    # frozen: both evaluation bags together are the gate; one development bag among them is not
    assert lap_src.guard_bags([tmp_path / n for n in names], TH, freeze) is True
    assert lap_src.guard_bags([tmp_path / names[0], tmp_path / "dev"], TH, freeze) is False
    assert lap_src.guard_bags([], TH, freeze) is False


# -- a bag written with mcap-ros2-support, the stream reader and the command line --------------------------

STRING_DEF = "string data\n"
HEADER = """std_msgs/Header header
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""
LAP_DEF = """std_msgs/Header header
uint32 lap_index
float32 lap_time_s
float32 lap_length_m
float32 v_cap_mps
bool clean
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""
ODOM_DEF = """std_msgs/Header header
geometry_msgs/TwistWithCovariance twist
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/TwistWithCovariance
geometry_msgs/Twist twist
================================================================================
MSG: geometry_msgs/Twist
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
"""


DRIVE_DEF = (
    "std_msgs/Header header\nackermann_msgs/AckermannDrive drive\n"
    "================================================================================\n"
    "MSG: ackermann_msgs/AckermannDrive\n"
    "float32 steering_angle\nfloat32 steering_angle_velocity\nfloat32 speed\n"
    "float32 acceleration\nfloat32 jerk\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n"
    "================================================================================\n"
    "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n"
)
PLANNER_DEF = (
    "std_msgs/Header header\nuint8 state\nfloat32 plan_min_clearance_m\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n"
    "================================================================================\n"
    "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n"
)


def write_bag(path: Path, scenario: Scenario | None = None) -> None:
    """The messages of a scenario in an mcap (only the topics lap-src D9-C0 reads)."""
    from mcap_ros2.writer import Writer

    sc = scenario or Scenario()
    with path.open("wb") as stream:
        w = Writer(stream, chunk_size=16 * 1024)
        s_str = w.register_msgdef("std_msgs/msg/String", STRING_DEF)
        s_lap = w.register_msgdef("racer_pnp_msgs/msg/LapEvent", LAP_DEF)
        s_odom = w.register_msgdef("nav_msgs/msg/Odometry", ODOM_DEF)
        s_drive = w.register_msgdef("ackermann_msgs/msg/AckermannDriveStamped", DRIVE_DEF)
        s_plan = w.register_msgdef("racer_pnp_msgs/msg/PlannerStatus", PLANNER_DEF)
        zero = {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}
        w.write_message("/scan", s_str, {"data": "scan"}, log_time=ns(0.0), publish_time=ns(0.0))
        for t_ns, topic, msg in sc.messages():
            if topic not in (
                "/odom/wheel",
                "/lap/event",
                "/lap/status",
                "/pnp/lap/event",
                "/pnp/lap/status",
                "/pnp/lap_shadow/event",
                "/pnp/lap_shadow/status",
                "/drive",
                "/drive_raw",
                "/pnp/planner/status",
            ):
                continue
            if topic in ("/drive", "/drive_raw"):
                w.write_message(
                    topic,
                    s_drive,
                    {
                        "header": zero,
                        "drive": {
                            "steering_angle": msg.drive.steering_angle,
                            "steering_angle_velocity": 0.0,
                            "speed": msg.drive.speed,
                            "acceleration": 0.0,
                            "jerk": 0.0,
                        },
                    },
                    log_time=t_ns,
                    publish_time=t_ns,
                )
            elif topic == "/pnp/planner/status":
                w.write_message(
                    topic,
                    s_plan,
                    {"header": zero, "state": msg.state, "plan_min_clearance_m": 0.3},
                    log_time=t_ns,
                    publish_time=t_ns,
                )
            elif topic == "/odom/wheel":
                w.write_message(
                    topic,
                    s_odom,
                    {
                        "header": {
                            "stamp": {
                                "sec": msg.header.stamp.sec,
                                "nanosec": msg.header.stamp.nanosec,
                            },
                            "frame_id": "",
                        },
                        "twist": {
                            "twist": {
                                "linear": {"x": msg.twist.twist.linear.x, "y": 0.0, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                            }
                        },
                    },
                    log_time=t_ns,
                    publish_time=t_ns,
                )
            elif topic in ("/lap/event", "/lap/status") or topic.endswith("/status"):
                w.write_message(topic, s_str, {"data": msg.data}, log_time=t_ns, publish_time=t_ns)
            elif topic.endswith("/event"):
                w.write_message(
                    topic,
                    s_lap,
                    {
                        "header": {
                            "stamp": {
                                "sec": msg.header.stamp.sec,
                                "nanosec": msg.header.stamp.nanosec,
                            },
                            "frame_id": "camera",
                        },
                        "lap_index": msg.lap_index,
                        "lap_time_s": msg.lap_time_s,
                        "lap_length_m": msg.lap_length_m,
                        "v_cap_mps": msg.v_cap_mps,
                        "clean": msg.clean,
                    },
                    log_time=t_ns,
                    publish_time=t_ns,
                )
        w.finish()


def test_a_bag_is_read_and_scored_end_to_end(tmp_path: Path) -> None:
    bag = tmp_path / "bag.mcap"
    write_bag(bag)
    data = lap_src.read_bag(bag)
    assert all(r.complete for r in data.reports) and data.reports[0].messages > 100
    assert len(data.cam_events) == 12 and len(data.nodes["/pnp/lap_shadow"].events) == 12
    block = lap_src.evaluate_c0(data, 12, TH)
    assert block["verdict"] == PASS, block["items"]
    assert data.t0_log_s == pytest.approx(T0_NS / 1e9)
    assert data.cam_events[0]["crossing_s"] == pytest.approx(Scenario().crossing(1), abs=1e-5)


def test_a_truncated_bag_is_read_up_to_the_break_and_says_so(tmp_path: Path) -> None:
    bag = tmp_path / "bag.mcap"
    write_bag(bag)
    cut = tmp_path / "cut.mcap"
    raw = bag.read_bytes()
    cut.write_bytes(raw[: int(len(raw) * 0.55)])
    rep = streamread.StreamReport()
    got = list(streamread.iter_stream(cut, [streamread.SCAN, "/lap/event", "/odom/wheel"], rep))
    assert got and got[0][1] == streamread.SCAN
    assert rep.complete is False and rep.stopped
    assert 0 < rep.bytes_read <= rep.file_bytes
    assert rep.last_log_time_ns is not None and rep.messages == len(got)
    assert "stopped at byte" in rep.say() and "cut.mcap" in rep.say()
    full = streamread.StreamReport()
    whole = list(streamread.iter_stream(bag, [streamread.SCAN, "/lap/event", "/odom/wheel"], full))
    assert full.complete and len(whole) > len(got)
    assert "read to the end" in full.say()
    # the lenient reader hands over what it read; the bag axis and the lap data follow from it
    data = lap_src.read_bag(cut)
    assert not data.reports[0].complete and 0 < len(data.cam_events) < 12
    out = io.StringIO()
    with redirect_stdout(out):
        pass
    t0, messages, reports = streamread.read_topics(cut, ["/lap/event"])
    assert t0 == ns(0.0) and all(m[1] == "/lap/event" for m in messages) and not reports[0].complete


def test_read_topics_needs_a_scan_and_a_file(tmp_path: Path) -> None:
    from mcap_ros2.writer import Writer

    empty = tmp_path / "noscan.mcap"
    with empty.open("wb") as stream:
        w = Writer(stream)
        s = w.register_msgdef("std_msgs/msg/String", STRING_DEF)
        w.write_message("/lap/event", s, {"data": "{}"}, log_time=1, publish_time=1)
        w.finish()
    with pytest.raises(ValueError, match="no /scan"):
        streamread.read_topics(empty, ["/lap/event"])
    with pytest.raises(FileNotFoundError):
        streamread.bag_files(tmp_path / "nothing")
    d = tmp_path / "bagdir"
    d.mkdir()
    with pytest.raises(FileNotFoundError):
        streamread.bag_files(d)
    (d / "a.mcap").write_bytes(empty.read_bytes())
    assert streamread.bag_files(d) == [d / "a.mcap"]


def run_cli(*argv: str) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        code = cli.main(["lap-src", *argv])
    return code, out.getvalue(), err.getvalue()


def test_the_command_line_scores_a_bag_and_writes_json(tmp_path: Path) -> None:
    bag = tmp_path / "bag.mcap"
    write_bag(bag)
    out = tmp_path / "report.json"
    code, text, _err = run_cli("--bag", str(bag), "--laps-counted", "12", "--out", str(out))
    assert code == 0 and "D9-C0 bag.mcap: PASS" in text
    report = json.loads(out.read_text())
    assert report["schema"] == "pnp-session-lap-src/1"
    assert report["prereg_yaml_sha256"] == lap_src.D9_YAML_SHA256
    assert report["d9_c0"][0]["verdict"] == PASS
    # a wrong count fails and exits non-zero; no count uses the wheel rule
    code, text, _err = run_cli("--bag", str(bag), "--laps-counted", "11")
    assert code == 1 and "FAIL" in text
    code, text, _err = run_cli("--bag", str(bag))
    assert code == 0 and "wheel rule" in text


def test_the_command_line_drive_table_and_the_refusals(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # the repo now carries the real freeze file; this test is about the unfrozen state, so point
    # the default freeze path at one that does not exist rather than depend on the repo's
    monkeypatch.setattr(lap_src, "D9_FREEZE_FILE", str(tmp_path / "no-such-freeze.json"))
    bag = tmp_path / "bag.mcap"
    write_bag(bag, Scenario(node_prefix="/pnp/lap"))
    code, text, _ = run_cli("--bag", str(bag), "--drive")
    assert code == 0 and "| lap | verdict |" in text
    nonode = tmp_path / "nonode.mcap"
    write_bag(nonode, Scenario(node_prefix=None))
    code, _text, err = run_cli("--bag", str(nonode))
    assert (
        code == cli.EXIT_PREREG and "REFUSED" in err and "no lap node on the camera source" in err
    )
    # an evaluation bag is refused before it is opened: the file does not even exist here
    code, _text, err = run_cli(
        "--bag", str(tmp_path / "2026-11-16T22-45-39_car_teleop"), "--offline"
    )
    assert code == cli.EXIT_PREREG and "not opened before the freeze" in err


def test_an_evaluation_bag_is_never_read_without_offline_even_when_frozen(tmp_path: Path) -> None:
    freeze = tmp_path / "b9.json"
    freeze.write_text(json.dumps({"commit": "a" * 40, "yaml_sha256": lap_src.D9_YAML_SHA256}))
    with pytest.raises(lap_src.FreezeRequired, match="--offline"):
        lap_src.run(
            [tmp_path / "2026-11-16T22-45-39_car_teleop"], freeze_path=freeze, offline=False
        )


def test_offline_through_the_command_line_is_labelled_a_regression(tmp_path: Path) -> None:
    bag = tmp_path / "bag.mcap"
    write_bag(bag, Scenario(n=13))
    lap_src.racer_laps_path()
    report, text, code = lap_src.run([bag], offline=True, odom=odom_model())
    assert "REGRESSION, NOT A GATE" in text and report["gate"] is False
    assert report["d9_b0"]["verdict"] == PASS and code == 0
    report, text, code = lap_src.run([bag], offline=True, assume_ready=True, odom=odom_model())
    assert report["assume_ready"] is True
    # an unfinished bag adds a NOTE
    cut = tmp_path / "cut.mcap"
    raw = bag.read_bytes()
    cut.write_bytes(raw[: int(len(raw) * 0.9)])
    _report, text, _code = lap_src.run([cut], offline=True, odom=odom_model())
    assert "NOTE: cut.mcap: stopped at byte" in text


def test_the_drive_table_shows_the_reference_suspect_flag_and_its_lap() -> None:
    bag = Scenario(node_prefix="/pnp/lap", node_suspect_lap=3).bag()
    table = lap_src.drive_table(bag)
    assert table["reference_suspect"] is True and table["reference_suspect_lap"] == 3
    text = lap_src.markdown_drive(table)
    assert "reference_suspect True (lap 3)" in text.splitlines()[0]
    clean = lap_src.drive_table(Scenario(node_prefix="/pnp/lap").bag())
    assert clean["reference_suspect"] is False and clean["reference_suspect_lap"] is None
    assert "reference_suspect False" in lap_src.markdown_drive(clean).splitlines()[0]
    # a status from before the flag existed: not known, not False
    bag = Scenario(node_prefix="/pnp/lap").bag()
    del bag.nodes["/pnp/lap"].statuses[-1][1]["reference_suspect"]
    assert lap_src.drive_table(bag)["reference_suspect"] is None
    assert (
        "reference_suspect unknown"
        in lap_src.markdown_drive(lap_src.drive_table(bag)).splitlines()[0]
    )
