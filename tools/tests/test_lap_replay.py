"""Tests for tools/lap_replay.py: the lap counter replayed over a bag or a folder of images.

The counter itself is tested in ros_ws/src/racer_perception/test/test_lap_counter.py. Here: that
the replay drives it the way the node does (rate limit, odometry, clock), the bag and folder
readers (a bag is written with mcap-ros2-support, trimmed message definitions with only the
fields the tool reads), the report and the command line. Tests that need real JPEG/PNG decoding
use OpenCV (the `vision` extra) and are skipped without it.
"""

from __future__ import annotations

import csv
import io
import math
from contextlib import redirect_stdout
from dataclasses import replace
from pathlib import Path

import lap_replay
import numpy as np
import pytest
from lap_replay import Msg, replay
from racer_perception.lap_counter import LapConfig, LapEvent

VIEW_W, VIEW_H = 64, 48
LOOP_PX = 400
PX_PER_FRAME = 4
FPS = 15.0
T0 = 5000.0  # bag clock start: nonzero so a stamp of 0 can never be mistaken for it


def make_world(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    w = rng.normal(size=(VIEW_H, LOOP_PX)).astype(np.float32)
    kx = np.ones(13, dtype=np.float32) / 13
    wide = np.concatenate([w, w, w], axis=1)
    wide = np.apply_along_axis(lambda r: np.convolve(r, kx, mode="same"), 1, wide)
    out = wide[:, LOOP_PX : 2 * LOOP_PX]
    out = (out - out.min()) / (out.max() - out.min())
    return (out * 200.0 + 20.0).astype(np.uint8)


def view(world: np.ndarray, pos: int) -> np.ndarray:
    return world[:, (pos + np.arange(VIEW_W)) % LOOP_PX]


def drive(laps: float, speed_mps: float = 1.0, fps: float = FPS, with_yaw: bool = True):
    """(messages, frames) of a car driving `laps` loops: odometry 50 Hz, yaw 50 Hz, frames fps.
    The frames are the raw gray arrays; the messages carry them as bytes."""
    world = make_world()
    msgs: list[Msg] = []
    n_frames = round(laps * LOOP_PX / PX_PER_FRAME)
    omega = 2 * np.pi / (LOOP_PX / PX_PER_FRAME / FPS)
    x = 0.0
    t_odom = T0
    for k in range(n_frames + 1):
        t = T0 + k / fps
        while t_odom <= t:
            x += speed_mps * 0.02
            msgs.append(Msg("odom", t_odom, (speed_mps, x)))
            if with_yaw:
                msgs.append(Msg("yaw", t_odom, float(omega)))
            t_odom += 0.02
        img = view(world, k * PX_PER_FRAME)
        msgs.append(Msg("image", t, img.tobytes()))
    return msgs


def raw_decode(data: bytes, _reduction: int):
    return np.frombuffer(data, dtype=np.uint8).reshape(VIEW_H, VIEW_W)


FAST = LapConfig(min_lap_time_s=3.0, min_lap_distance_m=1.0)


# ------------------------------------------------------------------------ replay()


def test_replay_counts_the_laps_a_drive_makes_and_records_the_trace():
    result = replay(drive(3.2), FAST, rate_hz=0.0, decode=raw_decode)
    assert [e.lap for e in result.events] == [1, 2, 3]
    assert result.frames_seen == result.frames_processed
    assert result.reference_time_s is not None and result.reference_time_s >= T0
    assert result.trace[0][1] == 1.0  # the reference frame itself
    assert max(s for _, s in result.trace[10:40]) < 0.8  # away from the start
    for e in result.events:
        assert e.heading_turns == pytest.approx(1.0, abs=0.06)
        assert e.distance_m == pytest.approx(e.lap_time_s, abs=0.2)
    assert result.final_status["laps"] == 3
    assert result.odom_messages > 100 and result.yaw_messages > 100


def test_replay_applies_the_frame_rate_limit_like_the_node():
    msgs = drive(1.3)  # 15 fps input
    full = replay(msgs, FAST, rate_hz=0.0, decode=raw_decode)
    limited = replay(msgs, FAST, rate_hz=10.0, decode=raw_decode)
    assert limited.frames_seen == full.frames_seen
    # S1: the throttle keeps a phase, so 10 Hz out of 15 fps is 10 of every 15 frames (the old
    # "period since the last accepted" rule delivered 7.5 Hz here)
    assert abs(limited.frames_processed - full.frames_seen * 10 / 15) <= 2


def test_replay_counts_undecodable_frames_and_keeps_going():
    msgs = drive(1.3)
    msgs.insert(60, Msg("image", T0 + 1.0, b"junk"))

    def decode(data, reduction):
        if data == b"junk":
            raise ValueError("bad")
        return raw_decode(data, reduction)

    result = replay(msgs, FAST, rate_hz=0.0, decode=decode)
    assert result.frames_failed == 1
    assert len(result.events) == 1


def test_replay_without_odometry_never_starts_with_the_default_start_gate():
    msgs = [m for m in drive(1.3) if m.kind == "image"]
    result = replay(msgs, FAST, rate_hz=0.0, decode=raw_decode)
    assert result.reference_time_s is None and result.trace == []
    assert result.final_status["state"] == "waiting"


def test_report_shows_unknown_turns_and_the_near_misses_of_a_gated_run():
    msgs = drive(2.3, with_yaw=False)
    counted = replay(msgs, FAST, rate_hz=0.0, decode=raw_decode)
    text = lap_replay.format_report(counted, "s", [], 1.0)
    assert "   --" in text.split("laps counted")[0]  # no yaw source: turns unknown
    assert "near misses" not in text
    # A minimum lap time longer than the drive blocks both passes of the line: they are listed.
    gated = replay(
        msgs, LapConfig(min_lap_time_s=100.0, min_lap_distance_m=1.0), 0.0, 4, raw_decode
    )
    gated_text = lap_replay.format_report(gated, "s", [], 1.0)
    assert "laps counted: 0" in gated_text and "near misses" in gated_text


def test_replay_rejects_an_unknown_message_kind():
    with pytest.raises(ValueError, match="unknown message kind"):
        replay([Msg("lidar", 0.0)], FAST)


def test_replay_of_nothing_is_empty_but_valid():
    result = replay([], FAST)
    assert result.events == [] and result.first_t is None and result.final_status["laps"] == 0


# ------------------------------------------------------------------------ report


def test_near_misses_show_stretches_that_rose_but_never_counted():
    cfg = LapConfig()
    # departs, a bump to 0.7 (never reaches 0.8: a near miss), drops, then a real crossing
    trace = [(0.0, 1.0), (1.0, 0.4), (2.0, 0.6), (3.0, 0.7), (4.0, 0.62), (5.0, 0.3)]
    trace += [(6.0, 0.5), (7.0, 0.9), (8.0, 0.97), (9.0, 0.4)]
    event = LapEvent(1, 8.0, 8.0, 0.97, 8.0, 3.0, 1.0)
    assert lap_replay.near_misses(trace, [event], cfg) == [(3.0, 0.7)]
    assert lap_replay.near_misses(trace, [], cfg) == [(3.0, 0.7), (8.0, 0.97)]
    # the stretch next to the reference before the car ever left is not a near miss
    assert lap_replay.near_misses([(0.0, 1.0), (1.0, 0.9), (2.0, 0.3)], [], cfg) == []
    # a stretch still open when the trace ends counts too
    assert lap_replay.near_misses([(0.0, 1.0), (1.0, 0.3), (2.0, 0.6)], [], cfg) == [(2.0, 0.6)]


def test_render_trace_marks_the_threshold_and_the_laps():
    result = replay(drive(2.3), FAST, rate_hz=0.0, decode=raw_decode)
    lines = lap_replay.render_trace(result, 1.0)
    assert "similarity_threshold 0.8" in lines[0]
    assert any("<- lap 1" in line for line in lines)
    assert any("<- lap 2" in line for line in lines)
    row = next(line for line in lines if "<- lap 1" in line)
    assert row.count("|") <= 1 and "#" in row


def test_render_trace_with_gaps_and_without_a_reference():
    assert (
        "no similarity samples" in lap_replay.render_trace(lap_replay.ReplayResult(LapConfig()))[0]
    )
    result = lap_replay.ReplayResult(LapConfig(), reference_time_s=0.0)
    result.trace = [(0.0, 1.0), (3.2, 0.5)]
    lines = lap_replay.render_trace(result, 1.0)
    assert sum(line.rstrip().endswith("--") for line in lines) == 2  # buckets 1 and 2 are empty


def test_format_report_names_what_happened():
    result = replay(drive(2.3), FAST, rate_hz=0.0, decode=raw_decode)
    text = lap_replay.format_report(result, "synthetic", ["a note"], 1.0)
    assert "source: synthetic" in text and "a note" in text
    assert "laps counted: 2" in text
    assert "lap  lap_time_s" in text and "similarity_threshold=0.8" in text
    empty = lap_replay.format_report(replay([], FAST), "nothing", [], 1.0)
    assert "no reference captured" in empty and "laps counted: 0" in empty


def test_write_trace_csv(tmp_path):
    result = replay(drive(1.3), FAST, rate_hz=0.0, decode=raw_decode)
    path = tmp_path / "trace.csv"
    lap_replay.write_trace_csv(path, result)
    rows = list(csv.DictReader(path.open()))
    assert len(rows) == len(result.trace)
    assert [r["lap_crossing"] for r in rows if r["lap_crossing"]] == ["1"]


# ------------------------------------------------------------------------ parameters


def test_parse_params_overrides_with_the_field_types():
    cfg = lap_replay.parse_params(
        [
            "similarity_threshold=0.9",
            "min_lap_time_s=12",
            "blur=false",
            "thumb_width_px=32",
        ],
        LapConfig(),
    )
    assert cfg.similarity_threshold == 0.9 and cfg.min_lap_time_s == 12.0
    assert cfg.blur is False and cfg.thumb_width_px == 32 and isinstance(cfg.thumb_width_px, int)
    assert lap_replay.parse_params(["blur=1"], LapConfig()).blur is True


@pytest.mark.parametrize(
    "pair",
    [
        "nonsense=1",
        "similarity_threshold",
        "similarity_threshold=high",
        "blur=maybe",
        "thumb_width_px=1.5",
        "rearm_threshold=0.95",  # above the similarity threshold: the config refuses it
    ],
)
def test_parse_params_refuses_bad_input(pair):
    with pytest.raises(SystemExit):
        lap_replay.parse_params([pair], LapConfig())


# ------------------------------------------------------------------------ bags

IMAGE_DEF = """std_msgs/Header header
string format
uint8[] data
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

# Trimmed nav_msgs/Odometry: only the fields the tool reads (pose position, twist), same names.
ODOM_DEF = """std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
================================================================================
MSG: geometry_msgs/PoseWithCovariance
geometry_msgs/Pose pose
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
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
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""


def _time(t: float) -> dict[str, int]:
    sec = int(t)
    return {"sec": sec, "nanosec": round((t - sec) * 1e9)}


def _odom_msg(t: float, speed: float, x: float, yaw: float) -> dict:
    return {
        "header": {"stamp": _time(t), "frame_id": "odom"},
        "child_frame_id": "base_link",
        "pose": {"pose": {"position": {"x": x, "y": 0.0, "z": 0.0}}},
        "twist": {
            "twist": {
                "linear": {"x": speed, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": yaw},
            }
        },
    }


THUMB_DEF = """\
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""


def write_bag(
    path: Path,
    msgs: list[Msg],
    image_topics=("/camera/usb/image_raw/compressed",),
    wheel_topic="/odom/wheel",
    yaw_topic="/odom",
    finish: bool = True,
    encode=lambda data: list(data),
) -> None:
    from mcap_ros2.writer import Writer

    with path.open("wb") as stream:
        writer = Writer(stream, chunk_size=64 * 1024)
        image_schema = writer.register_msgdef("sensor_msgs/msg/CompressedImage", IMAGE_DEF)
        odom_schema = writer.register_msgdef("nav_msgs/msg/Odometry", ODOM_DEF)
        thumb_schema = writer.register_msgdef("sensor_msgs/msg/Image", THUMB_DEF)
        for m in msgs:
            log = round(m.t * 1e9)
            if m.kind == "thumb":
                h, w = m.data.shape
                writer.write_message(
                    "/lap/thumb",
                    thumb_schema,
                    {
                        "header": {"stamp": _time(0.0), "frame_id": "cam"},
                        "height": h,
                        "width": w,
                        "encoding": "mono8",
                        "is_bigendian": 0,
                        "step": w,
                        "data": list(m.data.tobytes()),
                    },
                    log_time=log,
                )
            elif m.kind == "image":
                for topic in image_topics:
                    writer.write_message(
                        topic,
                        image_schema,
                        {
                            "header": {"stamp": _time(0.0), "frame_id": "cam"},
                            "format": "jpeg",
                            "data": encode(m.data),
                        },
                        log_time=log,
                    )
            elif m.kind == "odom":
                writer.write_message(
                    wheel_topic,
                    odom_schema,
                    _odom_msg(m.t, m.data[0], m.data[1], 0.0),
                    log_time=log,
                )
            else:
                writer.write_message(
                    yaw_topic, odom_schema, _odom_msg(m.t, 0.0, 0.0, m.data), log_time=log
                )
        if finish:
            writer.finish()


def test_bag_topics_and_image_topic_choice(tmp_path):
    bag = tmp_path / "run.mcap"
    write_bag(bag, drive(0.3))
    counts = lap_replay.list_bag_topics(bag)
    assert counts["/camera/usb/image_raw/compressed"] == 31
    assert counts["/odom/wheel"] > 0 and counts["/odom"] > 0
    assert lap_replay.pick_image_topic(counts, None) == "/camera/usb/image_raw/compressed"
    assert lap_replay.pick_image_topic(counts, "/odom") == "/odom"  # an explicit topic is trusted
    with pytest.raises(SystemExit, match="not in the bag"):
        lap_replay.pick_image_topic(counts, "/camera/none/image_raw/compressed")
    with pytest.raises(SystemExit, match="no /camera"):
        lap_replay.pick_image_topic({"/odom": 1}, None)
    two = {"/camera/a/image_raw/compressed": 1, "/camera/b/image_raw/compressed": 1}
    with pytest.raises(SystemExit, match="several cameras"):
        lap_replay.pick_image_topic(two, None)


def test_replay_of_a_bag_matches_replay_of_the_same_messages(tmp_path):
    msgs = drive(2.3)
    bag = tmp_path / "run.mcap"
    write_bag(bag, msgs)
    from_bag = replay(
        lap_replay.iter_bag_messages(
            bag, "/camera/usb/image_raw/compressed", "/odom/wheel", "/odom"
        ),
        FAST,
        rate_hz=0.0,
        decode=raw_decode,
    )
    direct = replay(msgs, FAST, rate_hz=0.0, decode=raw_decode)
    assert [e.lap for e in from_bag.events] == [1, 2]
    assert [e.crossing_time_s for e in from_bag.events] == pytest.approx(
        [e.crossing_time_s for e in direct.events], abs=1e-6
    )
    assert from_bag.events[0].heading_turns == pytest.approx(
        direct.events[0].heading_turns, abs=0.01
    )


def test_a_bag_cut_off_by_a_killed_recorder_still_replays(tmp_path, capsys):
    bag = tmp_path / "killed.mcap"
    write_bag(bag, drive(2.3), finish=False)  # no footer, as after a killed recorder
    result = replay(
        lap_replay.iter_bag_messages(
            bag, "/camera/usb/image_raw/compressed", "/odom/wheel", "/odom"
        ),
        FAST,
        rate_hz=0.0,
        decode=raw_decode,
    )
    assert len(result.events) >= 1
    assert "no footer" in capsys.readouterr().err


def test_bag_directory_with_two_cameras_needs_a_choice(tmp_path):
    folder = tmp_path / "bagdir"
    folder.mkdir()
    write_bag(
        folder / "a_0.mcap",
        drive(0.2),
        image_topics=("/camera/usb/image_raw/compressed", "/camera/csi0/image_raw/compressed"),
    )
    with pytest.raises(SystemExit, match="several cameras"):
        lap_replay.main([str(folder)])


def test_a_bag_without_a_camera_is_reported(tmp_path):
    bag = tmp_path / "nocam.mcap"
    write_bag(bag, [m for m in drive(0.2) if m.kind != "image"])
    with pytest.raises(SystemExit, match="no /camera"):
        lap_replay.main([str(bag)])


def test_directory_without_mcap_is_an_error(tmp_path):
    (tmp_path / "empty").mkdir()
    with pytest.raises(FileNotFoundError, match="no .mcap"):
        lap_replay.list_bag_topics(tmp_path / "empty")


# ------------------------------------------------------------------------ folders and CSV


def test_odom_csv_rows_become_odom_and_yaw_messages(tmp_path):
    path = tmp_path / "odom.csv"
    path.write_text("t_s,speed_mps,x_m,yaw_rate_rps\n0.0,1.0,0.0,0.5\n0.1,1.0,,\n")
    msgs = lap_replay.read_odom_csv(path)
    assert [(m.kind, m.t) for m in msgs] == [("odom", 0.0), ("yaw", 0.0), ("odom", 0.1)]
    assert msgs[0].data == (1.0, 0.0) and msgs[1].data == 0.5 and msgs[2].data == (1.0, None)


def test_odom_csv_needs_its_columns(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text("t,speed\n0,1\n")
    with pytest.raises(SystemExit, match="header"):
        lap_replay.read_odom_csv(path)


def _write_frames(folder: Path, laps: float, fmt: str = "png") -> int:
    cv2 = pytest.importorskip("cv2")
    folder.mkdir()
    world = make_world()
    n = round(laps * LOOP_PX / PX_PER_FRAME) + 1
    for k in range(n):
        cv2.imwrite(str(folder / f"frame_{k:05d}.{fmt}"), view(world, k * PX_PER_FRAME))
    return n


def test_folder_messages_are_in_name_order_at_fps_with_odometry_merged(tmp_path):
    folder = tmp_path / "frames"
    n = _write_frames(folder, 0.1)
    odom = tmp_path / "odom.csv"
    odom.write_text("t_s,speed_mps,x_m\n0.0,1.0,0.0\n0.05,1.0,0.05\n0.5,1.0,0.5\n")
    msgs = list(lap_replay.iter_folder_messages(folder, 10.0, odom))
    images = [m for m in msgs if m.kind == "image"]
    assert len(images) == n and [m.t for m in images[:3]] == pytest.approx([0.0, 0.1, 0.2])
    times = [m.t for m in msgs]
    assert times == sorted(times)  # merged in clock order
    assert sum(m.kind == "odom" for m in msgs) == 3


def test_folder_errors(tmp_path):
    (tmp_path / "none").mkdir()
    with pytest.raises(SystemExit, match="no .jpg"):
        list(lap_replay.iter_folder_messages(tmp_path / "none", 10.0, None))
    with pytest.raises(SystemExit, match="fps"):
        list(lap_replay.iter_folder_messages(tmp_path, 0.0, None))


# ------------------------------------------------------------------------ command line


def run_main(argv: list[str]) -> str:
    buf = io.StringIO()
    with redirect_stdout(buf):
        assert lap_replay.main(argv) == 0
    return buf.getvalue()


def test_main_on_a_folder_without_odometry_switches_the_gates_off_and_counts(tmp_path):
    folder = tmp_path / "frames"
    _write_frames(folder, 2.3)
    out = run_main(
        [
            "--images",
            str(folder),
            "--fps",
            "15",
            "--rate-hz",
            "0",
            "--decode-reduction",
            "1",
            "--param",
            "min_lap_time_s=3",
            "--trace-csv",
            str(tmp_path / "t.csv"),
        ]
    )
    assert "no --odom-csv" in out and "laps counted: 2" in out
    assert (tmp_path / "t.csv").exists() and "trace written" in out


def test_main_on_a_bag_with_real_jpeg_frames(tmp_path):
    cv2 = pytest.importorskip("cv2")
    world = make_world()
    msgs = []
    for m in drive(2.3, with_yaw=False):
        if m.kind == "image":
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(VIEW_H, VIEW_W)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            assert ok
            m = Msg("image", m.t, buf.tobytes())
        msgs.append(m)
    assert world.shape == (VIEW_H, LOOP_PX)
    bag = tmp_path / "run.mcap"
    write_bag(bag, msgs)
    out = run_main(
        [
            str(bag),
            "--rate-hz",
            "0",
            "--decode-reduction",
            "1",
            "--param",
            "min_lap_time_s=3",
            "--param",
            "min_lap_distance_m=1",
        ]
    )
    assert "laps counted: 2" in out
    assert "heading turns not available" in out  # no /odom in this bag
    assert "/camera/usb/image_raw/compressed" in out


def test_main_argument_errors(tmp_path):
    with pytest.raises(SystemExit, match="either a bag or"):
        lap_replay.main([])
    with pytest.raises(SystemExit, match="either a bag or"):
        lap_replay.main([str(tmp_path), "--images", str(tmp_path)])
    with pytest.raises(SystemExit, match="no such bag"):
        lap_replay.main([str(tmp_path / "missing")])
    with pytest.raises(SystemExit, match="no such folder"):
        lap_replay.main(["--images", str(tmp_path / "missing")])


def test_main_on_a_bag_without_wheel_odometry_switches_the_start_gate_off(tmp_path):
    bag = tmp_path / "noodom.mcap"
    write_bag(bag, [m for m in drive(0.3) if m.kind == "image"])
    out = run_main([str(bag), "--rate-hz", "0"])
    # the payloads are raw gray bytes, not JPEG: every frame is counted as undecodable, and the
    # report still says what it knows
    assert "no /odom/wheel in the bag" in out and "undecodable" in out


# ----------------------------------------------------------------------- split runs, reference


def test_split_run_resolves_a_whole_run_from_the_registered_yaml():
    run = lap_replay.load_split_run("L0")
    assert run.role == "dev"
    assert run.bag.name == "2026-11-14T21-49-21_car_teleop"
    assert run.t_from == pytest.approx(1794692965.664 + 124.9)
    assert run.t_to == pytest.approx(1794692965.664 + 239.9)


def test_split_run_spans_the_parts_of_a_run_cut_in_two():
    run = lap_replay.load_split_run("X2")
    first = 1794694074.843
    assert run.role == "test"
    assert (run.t_from, run.t_to) == (pytest.approx(first + 446.7), pytest.approx(first + 517.1))


def test_split_run_refuses_an_unknown_run_and_a_changed_yaml(tmp_path):
    with pytest.raises(SystemExit, match="not a lap run"):
        lap_replay.load_split_run("G1a")
    edited = tmp_path / "pnp-v0.4.yaml"
    edited.write_bytes(lap_replay.PREREG_V04_PATH.read_bytes() + b"\n# edited\n")
    with pytest.raises(SystemExit, match="SHA-256"):
        lap_replay.load_split_run("L0", edited)


def test_clip_messages_keeps_the_window_inclusive():
    msgs = [Msg("odom", t, (0.0, None)) for t in (1.0, 2.0, 3.0, 4.0)]
    assert [m.t for m in lap_replay.clip_messages(msgs, 2.0, 3.0)] == [2.0, 3.0]


def test_reference_report_counts_matched_missed_and_false(tmp_path):
    def ev(n, t):
        return LapEvent(n, t, t, 0.9, float(n), 5.0, None)

    ref = tmp_path / "lapref.json"
    ref.write_text('{"lap_count": 3, "lap_times_s": [5.5, 5.6, 5.7]}')
    count, times = lap_replay.load_reference(ref)
    assert (count, times) == (3, [5.5, 5.6, 5.7])
    equal = lap_replay.reference_report([ev(1, 5.5), ev(2, 5.7), ev(3, 5.6)], count, times)
    assert "matched 3, missed 0, false 0" in equal[0]
    assert "max |d| 0.10" in equal[1]
    short = lap_replay.reference_report([ev(1, 11.1)], count, times)
    assert "matched 1, missed 2, false 0" in short[0] and len(short) == 1
    extra = lap_replay.reference_report([ev(i, 5.0) for i in range(5)], count, times)
    assert "matched 3, missed 0, false 2" in extra[0]


def test_main_split_run_rejects_a_bag_next_to_it():
    with pytest.raises(SystemExit, match="replaces"):
        lap_replay.main(["some.mcap", "--split-run", "L0"])


# ------------------------------------------------------------- thumbnails and the peak table

THUMB_CFG = replace(FAST, start_speed_mps=0.0)
FIXTURE = Path(__file__).parent / "fixtures" / "lap_thumbs_2026_11_22_obs3.npz"


def thumb_drive(laps: float, **kw):
    """The drive as the counter's own thumbnails: the frames are already 64 x 48."""
    return [
        Msg("thumb", m.t, raw_decode(m.data, 4)) if m.kind == "image" else m
        for m in drive(laps, **kw)
    ]


def test_thumbs_replay_like_images_and_are_not_rate_limited():
    msgs = drive(2.3)
    thumbs = thumb_drive(2.3)
    by_image = replay(msgs, FAST, rate_hz=0.0, decode=raw_decode)
    by_thumb = replay(thumbs, FAST, rate_hz=1.0)  # a 1 Hz limit would drop most images
    assert by_thumb.frames_processed == by_thumb.frames_seen == by_image.frames_seen
    assert by_thumb.frame_kind == "thumb" and by_image.frame_kind == "image"
    assert [e.crossing_time_s for e in by_thumb.events] == [
        e.crossing_time_s for e in by_image.events
    ]


def test_replay_keeps_frames_only_when_asked():
    r = replay(thumb_drive(1.3), FAST)
    assert r.frames == []
    r = replay(thumb_drive(1.3), FAST, keep_frames=True)
    assert len(r.frames) == r.frames_processed
    assert r.frames[-1].distance_m > r.frames[0].distance_m
    assert float(np.linalg.norm(r.frames[5].signature)) == pytest.approx(1.0, abs=1e-5)


def test_estimate_lap_length_folds_missed_laps_and_ignores_a_short_alias():
    assert lap_replay.estimate_lap_length([]) is None
    assert lap_replay.estimate_lap_length([0.0, math.nan]) is None
    assert lap_replay.estimate_lap_length([5.5, 5.6, 5.4]) == pytest.approx(5.5, abs=0.05)
    assert lap_replay.estimate_lap_length([5.6, 11.0]) == pytest.approx(5.5, abs=0.15)
    assert lap_replay.estimate_lap_length([4.2, 5.5, 5.5, 11.1, 5.6]) == pytest.approx(5.5, abs=0.1)


def test_peak_table_finds_every_crossing_and_the_off_mark_level():
    result = replay(thumb_drive(4.2), THUMB_CFG, keep_frames=True)
    table = lap_replay.peak_table(result)
    assert table is not None
    assert [c.lap for c in table.crossings] == [1, 2, 3, 4]
    assert table.lap_length_m == pytest.approx(LOOP_PX / PX_PER_FRAME * 0.0667, rel=0.1)
    for c in table.crossings:
        assert c.standing_peak > 0.95
    assert table.crossings[0].driving_peak is None
    assert all(c.driving_peak > 0.95 for c in table.crossings[1:])
    hi, p99, n = table.off_mark["standing"]
    assert hi < 0.8 and p99 <= hi and n > 20
    assert table.peaks("standing") == [c.standing_peak for c in table.crossings]
    assert len(table.peaks("driving")) == 3


def test_peak_table_takes_the_lap_length_and_first_crossing_from_the_arguments():
    result = replay(thumb_drive(3.2), THUMB_CFG, keep_frames=True)
    auto = lap_replay.peak_table(result)
    given = lap_replay.peak_table(
        result, lap_length_m=auto.lap_length_m, first_station_m=auto.first_station_m
    )
    assert [c.station_m for c in given.crossings] == pytest.approx(
        [c.station_m for c in auto.crossings]
    )
    # a first crossing placed 3 m before the real one finds the best frame inside 0.7 m of it
    off = lap_replay.peak_table(result, auto.lap_length_m, auto.first_station_m - 3.0)
    assert off.crossings[0].standing_peak < 0.8


def test_peak_table_is_none_without_a_reference_or_a_way_to_place_crossings():
    nothing = replay([m for m in thumb_drive(1.3) if m.kind != "odom"], FAST, keep_frames=True)
    assert lap_replay.peak_table(nothing) is None
    gated = replay(
        thumb_drive(1.3),
        replace(THUMB_CFG, min_lap_time_s=100.0),
        keep_frames=True,
    )
    assert gated.events == [] and lap_replay.peak_table(gated) is None
    assert lap_replay.peak_table(gated, 5.0, 5.0) is not None


def test_render_peak_table_says_exact_or_approximate_and_flags_a_miss():
    result = replay(thumb_drive(3.2), THUMB_CFG, keep_frames=True)
    table = lap_replay.peak_table(result)
    exact = "\n".join(lap_replay.render_peak_table(table, "thumb", None))
    assert "exact: the counter's own thumbnails" in exact and "APPROXIMATE" not in exact
    approx = "\n".join(lap_replay.render_peak_table(table, "image", 4.7))
    assert "APPROXIMATE" in approx and "4.7 frames/s" in approx
    table.crossings[1].standing_peak = 0.7
    text = "\n".join(lap_replay.render_peak_table(table, "thumb", None))
    assert "0.700 <" in text and "under the threshold 0.8: 1" in text
    assert "(reference)" in text and "margin" in text


def test_thumb_from_image_reads_mono8_and_refuses_the_rest():
    from types import SimpleNamespace

    arr = np.arange(12, dtype=np.uint8)
    ok = SimpleNamespace(encoding="mono8", height=3, width=3, step=4, data=bytes(arr))
    assert lap_replay.thumb_from_image(ok).tolist() == [[0, 1, 2], [4, 5, 6], [8, 9, 10]]
    assert lap_replay.thumb_from_image(SimpleNamespace(**{**vars(ok), "encoding": "rgb8"})) is None
    assert lap_replay.thumb_from_image(SimpleNamespace(**{**vars(ok), "data": b"\x00" * 5})) is None
    assert lap_replay.thumb_from_image(SimpleNamespace(**{**vars(ok), "height": 0})) is None


def test_a_bag_with_lap_thumb_is_replayed_from_it_exactly(tmp_path, capsys):
    msgs = thumb_drive(2.3)
    bag = tmp_path / "thumbs.mcap"
    write_bag(bag, msgs)
    counts = lap_replay.list_bag_topics(bag)
    assert counts["/lap/thumb"] > 0
    assert lap_replay.pick_image_topic(counts, None) == "/lap/thumb"  # preferred, no camera needed
    read = list(lap_replay.iter_bag_messages(bag, "/lap/thumb", "/odom/wheel", "/odom"))
    thumbs = [m for m in read if m.kind == "thumb"]
    assert len(thumbs) == sum(1 for m in msgs if m.kind == "thumb")
    assert thumbs[3].data.tolist() == [m for m in msgs if m.kind == "thumb"][3].data.tolist()
    code = lap_replay.main(
        [str(bag), "--param", "min_lap_time_s=3", "--param", "min_lap_distance_m=1", "--peak-table"]
    )
    out = capsys.readouterr().out
    assert code == 0 and "exact: the counter's own thumbnails" in out and "laps counted: 2" in out


def test_save_thumbs_round_trips_through_the_npz(tmp_path):
    msgs = thumb_drive(2.3)
    path = tmp_path / "t.npz"
    n = lap_replay.save_thumbs(path, msgs)
    assert n == sum(1 for m in msgs if m.kind == "thumb")
    assert lap_replay.thumbs_are_exact(path)
    back = list(lap_replay.iter_thumbs_messages(path))
    assert [m.t for m in back] == sorted(m.t for m in back)
    assert {m.kind for m in back} == {"thumb", "odom", "yaw"}
    a = replay(msgs, FAST)
    b = replay(back, FAST)
    assert [e.lap for e in b.events] == [e.lap for e in a.events]
    assert [e.crossing_time_s for e in b.events] == pytest.approx(
        [e.crossing_time_s for e in a.events]
    )
    assert b.events[0].heading_turns == pytest.approx(a.events[0].heading_turns, abs=0.01)


def test_save_thumbs_of_jpeg_frames_is_marked_approximate(tmp_path):
    cv2 = pytest.importorskip("cv2")
    world = make_world()
    _, buf = cv2.imencode(".png", view(world, 0))
    msgs = [Msg("odom", T0, (1.0, 0.0)), Msg("image", T0, buf.tobytes())]
    path = tmp_path / "a.npz"
    assert lap_replay.save_thumbs(path, msgs) == 1
    assert not lap_replay.thumbs_are_exact(path)
    first = next(m for m in lap_replay.iter_thumbs_messages(path) if m.kind == "thumb")
    assert first.data.shape == (VIEW_H, VIEW_W)


def test_a_thumbs_file_without_the_exact_flag_is_taken_as_approximate(tmp_path):
    path = tmp_path / "old.npz"
    np.savez(path, t_s=np.zeros(0))
    assert not lap_replay.thumbs_are_exact(path)


def test_main_thumbs_window_and_source_errors(tmp_path, capsys):
    path = tmp_path / "t.npz"
    lap_replay.save_thumbs(path, thumb_drive(3.2))
    args = ["--param", "start_speed_mps=0", "--param", "min_lap_time_s=3"]
    args += ["--param", "min_lap_distance_m=1"]
    assert lap_replay.main(["--thumbs", str(path), *args]) == 0
    out = capsys.readouterr().out
    assert "approximate" not in out and "laps counted: 3" in out
    # a window that ends after one lap
    assert lap_replay.main(["--thumbs", str(path), "--t-to", str(T0 + 7.0), *args]) == 0
    out = capsys.readouterr().out
    assert "window: log time -inf to" in out and "laps counted: 1" in out
    with pytest.raises(SystemExit, match="either a bag or"):
        lap_replay.main(["--thumbs", str(path), "--images", str(tmp_path)])
    with pytest.raises(SystemExit, match="no such file"):
        lap_replay.main(["--thumbs", str(tmp_path / "none.npz")])
    with pytest.raises(SystemExit, match="split-run replaces"):
        lap_replay.main(["--split-run", "L0", "--thumbs", str(path)])


def test_main_peak_table_message_when_it_cannot_place_crossings(tmp_path, capsys):
    path = tmp_path / "t.npz"
    lap_replay.save_thumbs(path, thumb_drive(1.3))
    code = lap_replay.main(
        ["--thumbs", str(path), "--peak-table", "--param", "start_speed_mps=0"]
        + ["--param", "min_lap_time_s=100"]
    )
    assert code == 0 and "no crossings could be placed" in capsys.readouterr().out


def test_main_save_thumbs_writes_what_it_replayed(tmp_path, capsys):
    src = tmp_path / "src.npz"
    lap_replay.save_thumbs(src, thumb_drive(2.3))
    out = tmp_path / "out.npz"
    code = lap_replay.main(
        ["--thumbs", str(src), "--t-to", str(T0 + 5.0), "--save-thumbs", str(out)]
    )
    assert code == 0 and "thumbnails written to" in capsys.readouterr().out
    n = sum(1 for m in lap_replay.iter_thumbs_messages(out) if m.kind == "thumb")
    assert 70 < n < 80  # 5 s at 15 fps


# ---- L4: the thumbnails of the 2026-11-22 run 3 (ccw, 1.0 m/s, dim): a golden of the table


def test_fixture_thumbs_replay_and_peak_table_match_the_session_numbers(capsys):
    assert not lap_replay.thumbs_are_exact(FIXTURE)  # made from the record stream (5 fps)
    result = replay(
        lap_replay.iter_thumbs_messages(FIXTURE),
        LapConfig(start_speed_mps=0.0),
        keep_frames=True,
    )
    result.frame_kind = "image"
    assert result.frames_processed == 97
    # the counter, as on the car (the launch's reset: the first frame is the reference)
    assert [e.lap for e in result.events] == [1, 2]
    assert [e.peak_similarity for e in result.events] == pytest.approx([0.840, 0.937], abs=0.005)
    assert [e.distance_m for e in result.events] == pytest.approx([5.62, 11.02], abs=0.05)
    table = lap_replay.peak_table(result)
    assert table is not None
    assert table.lap_length_m == pytest.approx(5.6, abs=0.1)
    assert [c.lap for c in table.crossings] == [1, 2, 3]
    assert [c.standing_peak for c in table.crossings] == pytest.approx(
        [0.840, 0.778, 0.937], abs=0.005
    )
    assert [c.driving_peak for c in table.crossings[1:]] == pytest.approx([0.700, 0.875], abs=0.005)
    assert table.off_mark["standing"][0] == pytest.approx(0.736, abs=0.01)
    assert table.off_mark["driving"][0] == pytest.approx(0.760, abs=0.01)
    # the middle crossing is a peak under 0.80: the counter missed it (a double lap)
    assert table.crossings[1].standing_peak < table.threshold
    # the driving reference does not lift the weakest crossing here (checklist 3.5)
    assert min(table.peaks("driving")) < min(table.peaks("standing"))
    text = "\n".join(lap_replay.render_peak_table(table, result.frame_kind, 4.7))
    assert "APPROXIMATE" in text and "0.778 <" in text
