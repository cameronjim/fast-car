"""L1 unit tests for racer_drivers.rail_voltage (claude-docs/12-testing.md L1).

No ROS, no hardware: fixture strings for the hwmon attribute contents and a fixture tree of
ordinary files for discovery. The behaviour under test is the one CLAUDE.md invariant 5
depends on -- correct SI conversion of the hwmon milli-units, and never raising when the
sensor is absent or the files are garbage, because a logging driver that throws would take
the command path's process tree with it (claude-docs/05-safety.md: layer 4 must not weaken
layers 1-3).

Run by the l3-and-cpp CI job via colcon test (racer_drivers/CMakeLists.txt wires it with
ament_add_pytest_test); .github/scripts/run_python_tests.sh deliberately skips racer_drivers.
"""

from __future__ import annotations

import pathlib

import pytest
from racer_drivers import rail_voltage

# Verbatim shapes of the attribute contents the ina3221 hwmon driver writes: an integer in
# milli-units plus a trailing newline.
_VDD_IN_MILLIVOLTS = "11952\n"
_VDD_IN_MILLIAMPS = "1440\n"


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        (_VDD_IN_MILLIVOLTS, 11.952),
        ("0\n", 0.0),
        ("5000", 5.0),
        ("  4832  \n", 4.832),
        ("11952.0\n", 11.952),
    ],
)
def test_parse_millivolts_converts_to_volts(text: str, expected: float) -> None:
    assert rail_voltage.parse_millivolts(text) == pytest.approx(expected)


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        (_VDD_IN_MILLIAMPS, 1.44),
        ("0\n", 0.0),
        # The hwmon curr*_input files are signed; a negative reading must survive, not clamp.
        ("-240\n", -0.24),
    ],
)
def test_parse_milliamps_converts_to_amps(text: str, expected: float) -> None:
    assert rail_voltage.parse_milliamps(text) == pytest.approx(expected)


@pytest.mark.parametrize("text", [None, "", "   \n", "n/a\n", "0x2eb0\n", "11952 mV\n"])
def test_unparseable_values_are_none_not_exceptions(text: str | None) -> None:
    assert rail_voltage.parse_millivolts(text) is None
    assert rail_voltage.parse_milliamps(text) is None


@pytest.mark.parametrize(
    ("text", "expected"),
    [("VDD_IN\n", "VDD_IN"), ("  VDD_SOC  \n", "VDD_SOC"), ("", None), ("\n", None), (None, None)],
)
def test_parse_label(text: str | None, expected: str | None) -> None:
    assert rail_voltage.parse_label(text) == expected


@pytest.mark.parametrize(
    ("label", "expected"),
    [
        ("VDD_IN", "vdd_in"),
        ("VDD_CPU_GPU_CV", "vdd_cpu_gpu_cv"),
        ("VDD IN", "vdd_in"),
        ("VDD-IN+5V", "vdd_in_5v"),
        ("___", "unlabelled"),
    ],
)
def test_slugify_label_is_topic_safe(label: str, expected: str) -> None:
    assert rail_voltage.slugify_label(label) == expected


def _write_fixture_hwmon(root: pathlib.Path) -> pathlib.Path:
    """An Orin-Nano-shaped ina3221 tree: bus-address dir, hwmon dir, three labelled rails."""
    hwmon = root / "1-0040" / "hwmon" / "hwmon3"
    hwmon.mkdir(parents=True)
    channels = {
        1: ("VDD_IN", _VDD_IN_MILLIVOLTS, _VDD_IN_MILLIAMPS),
        2: ("VDD_CPU_GPU_CV", "4832\n", "560\n"),
        3: ("VDD_SOC", "3296\n", "912\n"),
    }
    for index, (label, millivolts, milliamps) in channels.items():
        (hwmon / f"in{index}_label").write_text(f"{label}\n", encoding="utf-8")
        (hwmon / f"in{index}_input").write_text(millivolts, encoding="utf-8")
        (hwmon / f"curr{index}_input").write_text(milliamps, encoding="utf-8")
    return hwmon


def test_find_hwmon_dir_globs_the_enumeration_dependent_path(tmp_path: pathlib.Path) -> None:
    expected = _write_fixture_hwmon(tmp_path)
    assert rail_voltage.find_hwmon_dir(str(tmp_path)) == str(expected)


def test_find_hwmon_dir_returns_none_when_the_sensor_is_not_fitted(
    tmp_path: pathlib.Path,
) -> None:
    assert rail_voltage.find_hwmon_dir(str(tmp_path / "nothing-here")) is None
    (tmp_path / "1-0040").mkdir()  # driver dir present, no hwmon child
    assert rail_voltage.find_hwmon_dir(str(tmp_path)) is None


def test_discover_channels_orders_by_index_and_keeps_labels(tmp_path: pathlib.Path) -> None:
    hwmon = _write_fixture_hwmon(tmp_path)
    channels = rail_voltage.discover_channels(str(hwmon))
    assert [(channel.index, channel.label) for channel in channels] == [
        (1, "VDD_IN"),
        (2, "VDD_CPU_GPU_CV"),
        (3, "VDD_SOC"),
    ]
    assert [channel.slug for channel in channels] == ["vdd_in", "vdd_cpu_gpu_cv", "vdd_soc"]


def test_discover_channels_skips_unlabelled_and_voltage_less_entries(
    tmp_path: pathlib.Path,
) -> None:
    hwmon = _write_fixture_hwmon(tmp_path)
    # A shunt-voltage attribute with no label, as some kernels expose.
    (hwmon / "in4_input").write_text("12\n", encoding="utf-8")
    # A labelled channel with no in*_input: not a rail this node can report.
    (hwmon / "in5_label").write_text("VDD_GHOST\n", encoding="utf-8")
    # An empty label file.
    (hwmon / "in6_label").write_text("\n", encoding="utf-8")
    (hwmon / "in6_input").write_text("1000\n", encoding="utf-8")
    assert [channel.index for channel in rail_voltage.discover_channels(str(hwmon))] == [1, 2, 3]


def test_discover_channels_on_a_missing_directory_is_empty_not_an_error(
    tmp_path: pathlib.Path,
) -> None:
    assert rail_voltage.discover_channels(str(tmp_path / "gone")) == []


def test_read_all_returns_si_values(tmp_path: pathlib.Path) -> None:
    hwmon = _write_fixture_hwmon(tmp_path)
    channels = rail_voltage.discover_channels(str(hwmon))
    readings = rail_voltage.read_all(str(hwmon), channels)
    assert [reading.channel.label for reading in readings] == [
        "VDD_IN",
        "VDD_CPU_GPU_CV",
        "VDD_SOC",
    ]
    assert readings[0].volts == pytest.approx(11.952)
    assert readings[0].amps == pytest.approx(1.44)
    assert readings[2].volts == pytest.approx(3.296)


def test_read_channel_tolerates_a_missing_current_file(tmp_path: pathlib.Path) -> None:
    hwmon = _write_fixture_hwmon(tmp_path)
    (hwmon / "curr1_input").unlink()
    reading = rail_voltage.read_channel(str(hwmon), rail_voltage.RailChannel(1, "VDD_IN"))
    assert reading.volts == pytest.approx(11.952)
    assert reading.amps is None


def test_read_channel_tolerates_a_garbage_voltage_file(tmp_path: pathlib.Path) -> None:
    hwmon = _write_fixture_hwmon(tmp_path)
    (hwmon / "in1_input").write_text("", encoding="utf-8")
    reading = rail_voltage.read_channel(str(hwmon), rail_voltage.RailChannel(1, "VDD_IN"))
    assert reading.volts is None
    assert reading.amps == pytest.approx(1.44)


def test_read_text_on_a_missing_file_is_none(tmp_path: pathlib.Path) -> None:
    assert rail_voltage.read_text(str(tmp_path / "in9_input")) is None
