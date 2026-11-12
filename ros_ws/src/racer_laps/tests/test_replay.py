"""L1: the bag replay tool on synthetic messages and on a tiny mcap written here."""

from __future__ import annotations

import math
from collections.abc import Iterator
from pathlib import Path
from types import SimpleNamespace as NS
from typing import Any

import pytest
from racer_laps.replay import (
    Pose2D,
    TfTree,
    format_report,
    main,
    quat_yaw,
    read_messages,
    run_replay,
)
from tracks import circle_poses

TWO_PI = 2.0 * math.pi


def tf_msg(parent: str, child: str, x: float, y: float, yaw: float) -> Any:
    tr = NS(
        header=NS(frame_id=parent),
        child_frame_id=child,
        transform=NS(
            translation=NS(x=x, y=y, z=0.0),
            rotation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
        ),
    )
    return NS(transforms=[tr])


def stream(
    loops: float, odom_offset: tuple[float, float, float] = (0.0, 0.0, 0.0), gap: tuple = (1e9, 1e9)
) -> Iterator[tuple[float, str, Any]]:
    """map -> odom (static offset) and odom -> base_link on a circle, 50 Hz of /tf."""
    ox, oy, oyaw = odom_offset
    yield 0.0, "/tf_static", tf_msg("map", "odom", ox, oy, oyaw)
    poses = list(circle_poses(loops, rate_hz=50.0))
    for t, x, y, yaw in poses:
        if gap[0] < t < gap[1]:
            continue
        yield t + 1.0, "/tf", tf_msg("odom", "base_link", x, y, yaw)


def test_pose2d_compose_and_inverse() -> None:
    a = Pose2D(1.0, 2.0, math.pi / 2)
    b = Pose2D(3.0, 0.0, 0.2)
    c = a.compose(b)
    assert (c.x, c.y) == pytest.approx((1.0, 5.0))
    assert c.yaw == pytest.approx(math.pi / 2 + 0.2)
    back = a.compose(a.inverse())
    assert (back.x, back.y, back.yaw) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert quat_yaw(0.0, 0.0, math.sin(0.4), math.cos(0.4)) == pytest.approx(0.8)


def test_tf_tree_chain_inverse_age_and_missing() -> None:
    tree = TfTree()
    assert tree.lookup("map", "base_link", 0.0) is None
    tree.update("map", "odom", Pose2D(1.0, 0.0, 0.0), 0.0, static=True)
    tree.update("/base_link", "/laser", Pose2D(0.0, 0.5, 0.0), 5.0, static=False)
    tree.update("odom", "base_link", Pose2D(2.0, 0.0, 0.0), 10.0, static=False)
    found = tree.lookup("map", "base_link", 10.25)
    assert found is not None
    pose, age = found
    assert (pose.x, pose.y) == pytest.approx((3.0, 0.0))
    assert age == pytest.approx(0.25)  # the static edge adds nothing
    # a child looked up in the opposite direction goes through an inverse
    inv = tree.lookup("laser", "map", 10.25)
    assert inv is not None
    assert (inv[0].x, inv[0].y) == pytest.approx((-3.0, -0.5))


@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0), (5.0, -3.0, 1.0)])
def test_replay_finds_one_lap_per_loop_whatever_the_map_offset(
    offset: tuple[float, float, float],
) -> None:
    result = run_replay(stream(3.1, odom_offset=offset))
    assert [lap.lap_index for lap in result.laps] == [1, 2, 3]
    assert all(abs(lap.lap_time_s - TWO_PI) < 0.1 for lap in result.laps)
    assert not result.saw_planner_status
    assert not any(lap.clean for lap in result.laps)  # no clearance data in the bag
    text = format_report(result)
    assert "3 laps" in text and "no /pnp/planner/status" in text


def test_replay_uses_planner_status_and_safety_events() -> None:
    def msgs() -> Iterator[tuple[float, str, Any]]:
        yield 0.0, "/pnp/planner/status", NS(state=1, plan_min_clearance_m=0.3)
        braked = False
        for t, topic, m in stream(2.1):
            yield t, topic, m
            if t > 1.0 + 9.0 and not braked:
                braked = True
                yield t, "/safety/events", NS(phase=0, severity=2)
            if int(t * 50) % 5 == 0 and topic == "/tf":
                yield t, "/pnp/planner/status", NS(state=1, plan_min_clearance_m=0.3)
                yield t, "/odom/wheel", NS(twist=NS(twist=NS(linear=NS(x=1.0))))
                yield t, "/drive_raw", NS(drive=NS(speed=1.0, steering_angle=0.0))

    result = run_replay(msgs())
    assert result.saw_planner_status
    assert [lap.clean for lap in result.laps] == [True, False]
    assert result.laps[1].safety_brakes == 1
    assert result.safe_set_laps == 1


def test_replay_reports_missing_pose_and_no_lap_through_a_dropout() -> None:
    result = run_replay(stream(2.1, gap=(2.0, 3.0)))
    assert [lap.lap_index for lap in result.laps] == [1, 2]
    assert result.n_pose_missing > 5  # the 1 s of silence beyond the 0.5 s timeout

    empty = run_replay(iter([(0.0, "/tf_static", tf_msg("a", "b", 0.0, 0.0, 0.0))]))
    assert empty.n_pose_samples == 0
    assert "no usable map -> base_link" in format_report(empty)


def test_read_messages_errors_on_a_bad_path(tmp_path: Path) -> None:
    pytest.importorskip("mcap")
    with pytest.raises(FileNotFoundError):
        list(read_messages(tmp_path / "nope"))
    with pytest.raises(FileNotFoundError):
        list(read_messages(tmp_path))  # a directory with no .mcap


TFMESSAGE_DEF = """geometry_msgs/TransformStamped[] transforms
================================================================================
MSG: geometry_msgs/TransformStamped
std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Transform
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
"""


def test_replay_from_a_real_mcap_file(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    pytest.importorskip("mcap_ros2")
    from mcap_ros2.writer import Writer

    def tf_dict(parent: str, child: str, x: float, y: float, yaw: float, t: float) -> dict:
        return {
            "transforms": [
                {
                    "header": {
                        "stamp": {"sec": int(t), "nanosec": int((t % 1) * 1e9)},
                        "frame_id": parent,
                    },
                    "child_frame_id": child,
                    "transform": {
                        "translation": {"x": x, "y": y, "z": 0.0},
                        "rotation": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": math.sin(yaw / 2),
                            "w": math.cos(yaw / 2),
                        },
                    },
                }
            ]
        }

    bag = tmp_path / "bag"
    bag.mkdir()
    with (bag / "bag_0.mcap").open("wb") as f:
        writer = Writer(f)
        schema = writer.register_msgdef("tf2_msgs/msg/TFMessage", TFMESSAGE_DEF)
        for t, x, y, yaw in circle_poses(2.1, rate_hz=50.0):
            writer.write_message(
                "/tf",
                schema,
                tf_dict("map", "base_link", x, y, yaw, t + 100.0),
                log_time=int((t + 100.0) * 1e9),
                publish_time=int((t + 100.0) * 1e9),
                sequence=0,
            )
        writer.finish()
    result = run_replay(read_messages(bag))
    assert [lap.lap_index for lap in result.laps] == [1, 2]
    assert main(["--bag", str(bag)]) == 0
    assert "2 laps" in capsys.readouterr().out


def test_main_exits_nonzero_when_the_bag_has_no_pose(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    pytest.importorskip("mcap_ros2")
    from mcap_ros2.writer import Writer

    bag = tmp_path / "bag.mcap"
    with bag.open("wb") as f:
        writer = Writer(f)
        schema = writer.register_msgdef("tf2_msgs/msg/TFMessage", TFMESSAGE_DEF)
        writer.write_message("/other", schema, {"transforms": []}, log_time=1, publish_time=1)
        writer.finish()
    assert main(["--bag", str(bag)]) == 1
    assert "no usable map -> base_link" in capsys.readouterr().out
