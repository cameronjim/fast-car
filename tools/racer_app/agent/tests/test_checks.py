from __future__ import annotations

import pytest
from fakes import healthy_ros
from racer_agent import checks as ck
from racer_agent.config import Thresholds
from racer_agent.rosstate import RosState

T = Thresholds()


def ros(voltage=12.0, scan_hz=10.0, fault="NONE", fet=35.0, motor=0.0, connected=True, drive=True):
    state = RosState()
    healthy_ros(state, scan_hz=scan_hz, voltage=voltage)
    state.set_value("fault", fault)
    state.set_value("temp_fet_degc", fet)
    state.set_value("temp_motor_degc", motor)
    if not drive:
        state.set_topics({"/scan"})
    if not connected:
        state.set_connected(False)
    return state.snapshot()


def snap(**kw):
    base = {
        "stack_running": True,
        "recorder_running": True,
        "containers": {"car-stack": "running"},
        "stack_flags": {"lidar": "true", "vesc": "true"},
        "disk_free_gb": 50.0,
        "lidar_device": True,
        "vesc_device": True,
        "heartbeat_active": True,
        "zones_c": {"cpu-thermal": 45.0, "gpu-thermal": 44.0},
        "wifi_dbm": -50.0,
        "camera_devices": ["/dev/video1"],
        "uptime_s": 3600.0,
        "ros": ros(),
    }
    base.update(kw)
    return ck.Snapshot(**base)


def by_id(report, check_id):
    return next(c for c in report.checks if c.id == check_id)


def test_everything_healthy_is_green_and_ready():
    report = ck.evaluate(snap(), T)
    assert report.overall == ck.GREEN and report.all_required_green
    assert report.as_dict()["ready"] is True
    assert not report.hard_failures


@pytest.mark.parametrize(
    "volts,level",
    [
        (12.6, ck.GREEN),
        (11.1, ck.GREEN),  # exactly 3.70 V per cell
        (11.09, ck.AMBER),
        (10.5, ck.AMBER),  # exactly 3.50 V per cell
        (10.49, ck.RED),
        (9.0, ck.RED),
        (5.0, ck.RED),  # no pack, USB powered
        (13.0, ck.AMBER),  # 4.33 V per cell: is it really 3S?
    ],
)
def test_battery_levels_for_a_3s_pack(volts, level):
    assert ck.check_battery(snap(ros=ros(voltage=volts)), T).level == level


def test_battery_shows_per_cell_voltage():
    assert "4.00 V per cell" in ck.check_battery(snap(ros=ros(voltage=12.0)), T).detail


def test_battery_without_reading_is_red():
    assert ck.check_battery(snap(ros=None), T).level == ck.RED
    state = RosState()
    state.set_connected(True)
    assert ck.check_battery(snap(ros=state.snapshot()), T).level == ck.RED


def test_battery_reading_goes_stale():
    state = RosState(clock=lambda: now[0])
    now = [0.0]
    state.set_connected(True)
    state.set_value("voltage_v", 12.0)
    now[0] = 10.0
    assert ck.check_battery(snap(ros=state.snapshot()), T).level == ck.RED


def test_cell_count_is_configurable():
    t = Thresholds(battery_cells=2)
    assert ck.check_battery(snap(ros=ros(voltage=7.6)), t).level == ck.GREEN


@pytest.mark.parametrize(
    "free,level",
    [
        (100.0, ck.GREEN),
        (15.0, ck.GREEN),
        (14.9, ck.AMBER),
        (5.0, ck.AMBER),
        (4.99, ck.RED),
        (0.0, ck.RED),
    ],
)
def test_disk_levels(free, level):
    c = ck.check_disk(snap(disk_free_gb=free), T)
    assert c.level == level and c.hard


def test_disk_unreadable_is_red():
    assert ck.check_disk(snap(disk_free_gb=None), T).level == ck.RED


@pytest.mark.parametrize(
    "hz,level", [(10.0, ck.GREEN), (5.0, ck.GREEN), (4.0, ck.AMBER), (1.0, ck.AMBER), (0.0, ck.RED)]
)
def test_lidar_scan_rate_levels(hz, level):
    assert ck.check_lidar(snap(ros=ros(scan_hz=hz)), T).level == level


def test_lidar_red_without_device_or_flag_or_ros():
    assert ck.check_lidar(snap(lidar_device=False), T).level == ck.RED
    assert ck.check_lidar(snap(stack_flags={"lidar": "false"}), T).level == ck.RED
    assert ck.check_lidar(snap(ros=ros(connected=False)), T).level == ck.RED
    assert ck.check_lidar(snap(ros=None), T).level == ck.RED


def test_vesc_green_and_fault_red():
    assert ck.check_vesc(snap(), T).level == ck.GREEN
    for ok in ("", "none", "FAULT_CODE_NONE"):
        assert ck.check_vesc(snap(ros=ros(fault=ok)), T).level == ck.GREEN
    bad = ck.check_vesc(snap(ros=ros(fault="FAULT_CODE_OVER_TEMP_FET")), T)
    assert bad.level == ck.RED and "OVER_TEMP_FET" in bad.detail
    assert ck.check_vesc(snap(ros=ros(fault="STALE_NO_VESC_DATA")), T).level == ck.RED


def test_vesc_red_without_uart_flag_ros_or_telemetry():
    assert ck.check_vesc(snap(vesc_device=False), T).level == ck.RED
    assert ck.check_vesc(snap(stack_flags={"vesc": "false"}), T).level == ck.RED
    assert ck.check_vesc(snap(ros=None), T).level == ck.RED
    empty = RosState()
    empty.set_connected(True)
    assert ck.check_vesc(snap(ros=empty.snapshot()), T).level == ck.RED


def test_safety_node_is_judged_by_the_drive_topic():
    assert ck.check_safety_node(snap()).level == ck.GREEN
    assert ck.check_safety_node(snap(ros=ros(drive=False))).level == ck.RED
    assert ck.check_safety_node(snap(ros=ros(connected=False))).level == ck.RED
    assert ck.check_safety_node(snap(ros=None)).level == ck.RED


def test_heartbeat_levels():
    assert ck.check_heartbeat(snap()).level == ck.GREEN
    assert ck.check_heartbeat(snap(heartbeat_active=False)).level == ck.RED
    assert ck.check_heartbeat(snap(heartbeat_active=None)).level == ck.AMBER


@pytest.mark.parametrize(
    "cpu,level",
    [(45.0, ck.GREEN), (69.9, ck.GREEN), (70.0, ck.AMBER), (84.9, ck.AMBER), (85.0, ck.RED)],
)
def test_cpu_gpu_temperature_levels(cpu, level):
    assert ck.check_temps(snap(zones_c={"cpu-thermal": cpu}), T).level == level


@pytest.mark.parametrize(
    "fet,level", [(40.0, ck.GREEN), (60.0, ck.AMBER), (79.9, ck.AMBER), (80.0, ck.RED)]
)
def test_vesc_fet_temperature_levels(fet, level):
    assert ck.check_temps(snap(ros=ros(fet=fet)), T).level == level


def test_motor_temp_zero_means_no_sensor_and_hot_motor_is_red():
    assert "motor" not in ck.check_temps(snap(ros=ros(motor=0.0)), T).detail
    assert ck.check_temps(snap(ros=ros(motor=90.0)), T).level == ck.RED
    assert ck.check_temps(snap(ros=ros(motor=65.0)), T).level == ck.AMBER


def test_worst_temperature_wins_and_no_readings_is_amber():
    c = ck.check_temps(snap(zones_c={"cpu-thermal": 50.0, "gpu-thermal": 90.0}), T)
    assert c.level == ck.RED and "gpu-thermal 90 C" in c.detail
    assert ck.check_temps(snap(zones_c={"soc-thermal": 90.0}, ros=None), T).level == ck.AMBER


def test_non_cpu_zones_are_ignored():
    assert (
        ck.check_temps(snap(zones_c={"cpu-thermal": 40.0, "tj-thermal": 99.0}), T).level == ck.GREEN
    )


@pytest.mark.parametrize(
    "dbm,level",
    [(-40.0, ck.GREEN), (-65.0, ck.GREEN), (-66.0, ck.AMBER), (-75.0, ck.AMBER), (-76.0, ck.RED)],
)
def test_wifi_levels_and_never_required(dbm, level):
    c = ck.check_wifi(snap(wifi_dbm=dbm), T)
    assert c.level == level and not c.required


def test_wired_is_green():
    assert ck.check_wifi(snap(wifi_dbm=None), T).level == ck.GREEN


def test_optional_checks_never_gate():
    s = snap(
        wifi_dbm=-90.0,
        containers={"car-stack": "running"},
        camera_devices=[],
        uptime_s=10.0,
    )
    report = ck.evaluate(s, T)
    assert by_id(report, "wifi").level == ck.RED
    assert by_id(report, "uptime").level == ck.AMBER
    assert by_id(report, "cameras").level == ck.AMBER
    assert report.all_required_green


def test_cameras_and_containers_and_uptime_details():
    running = snap(containers={"car-stack": "running", "car-camera": "running"})
    assert ck.check_cameras(running).level == ck.GREEN
    assert ck.check_containers(running).level == ck.GREEN
    assert ck.check_containers(snap(containers={"car-stack": "restarting"})).level == ck.AMBER
    assert ck.check_containers(snap(containers={})).level == ck.AMBER
    assert "min" in ck.check_uptime(snap(uptime_s=300.0), T).detail
    assert " h" in ck.check_uptime(snap(uptime_s=7200.0), T).detail
    assert ck.check_uptime(snap(uptime_s=None), T).level == ck.AMBER
    assert ck.check_uptime(snap(uptime_s=60.0), T).detail.endswith("just booted")


def test_no_stack_means_hard_failures():
    report = ck.evaluate(snap(stack_running=False, recorder_running=False), T)
    assert {c.id for c in report.hard_failures} == {"stack", "recorder"}
    assert report.overall == ck.RED


def test_stack_without_recorder_is_a_hard_failure():
    report = ck.evaluate(snap(recorder_running=False), T)
    assert [c.id for c in report.hard_failures] == ["recorder"]


def test_full_disk_is_a_hard_failure_but_low_battery_is_not():
    report = ck.evaluate(snap(disk_free_gb=2.0, ros=ros(voltage=9.0)), T)
    assert [c.id for c in report.hard_failures] == ["disk"]
    assert by_id(report, "battery").level == ck.RED and not by_id(report, "battery").hard


def test_not_green_required_lists_amber_and_red():
    report = ck.evaluate(snap(disk_free_gb=10.0, ros=ros(voltage=10.8)), T)
    assert {c.id for c in report.not_green_required} == {"disk", "battery"}


def test_worst_of_empty_is_green():
    assert ck.worst([]) == ck.GREEN
