"""install.sh and uninstall.sh, run for real against a temporary /etc and state directory."""

from __future__ import annotations

import getpass
import json
import os
import stat
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

DEPLOY = Path(__file__).resolve().parents[2] / "deploy"
INSTALL = DEPLOY / "install.sh"
UNINSTALL = DEPLOY / "uninstall.sh"


class Sandbox:
    def __init__(self, tmp_path: Path) -> None:
        self.etc = tmp_path / "etc"
        self.state = tmp_path / "state"
        self.systemctl_log = tmp_path / "systemctl.log"
        wrapper = tmp_path / "racer-agent"
        wrapper.write_text(f'#!/bin/sh\nexec "{sys.executable}" -m racer_agent "$@"\n')
        wrapper.chmod(0o755)
        fake_systemctl = tmp_path / "systemctl"
        fake_systemctl.write_text(f'#!/bin/sh\necho "$@" >> "{self.systemctl_log}"\n')
        fake_systemctl.chmod(0o755)
        self.env = {
            **os.environ,
            "RACER_SUDO": "",
            "RACER_ETC": str(self.etc),
            "RACER_STATE_DIR": str(self.state),
            "RACER_SYSTEMCTL": str(fake_systemctl),
            "RACER_USER": getpass.getuser(),
            "RACER_AGENT_BIN": str(wrapper),
            "RACER_SKIP_BUILD": "1",
        }

    def run(self, script: Path, *args: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["bash", str(script), *args], env=self.env, capture_output=True, text=True, check=False
        )

    @property
    def config(self) -> Path:
        return self.etc / "racer-app" / "config.json"

    def systemctl_calls(self) -> list[str]:
        return self.systemctl_log.read_text().splitlines() if self.systemctl_log.exists() else []


@pytest.fixture
def sb(tmp_path: Path) -> Sandbox:
    return Sandbox(tmp_path)


def token_in(output: str) -> str:
    return output.split("save it now):")[1].split()[0]


def test_first_install_lays_everything_down(sb):
    r = sb.run(INSTALL)
    assert r.returncode == 0, r.stderr
    assert "RACER APP TOKEN" in r.stdout and "Save that token now" in r.stdout
    token = token_in(r.stdout)
    raw = json.loads(sb.config.read_text())
    assert raw["token_sha256"] and token not in sb.config.read_text()
    assert stat.S_IMODE(sb.config.stat().st_mode) == 0o600
    assert (sb.state / "autostart.enabled").exists()  # autostart defaults to ON
    assert "Autostart: ON" in r.stdout


def test_units_are_rendered_with_real_paths_and_no_placeholders(sb):
    sb.run(INSTALL)
    repo = str(DEPLOY.parents[2])
    agent = (sb.etc / "systemd/system/racer-agent.service").read_text()
    stack = (sb.etc / "systemd/system/racer-stack.service").read_text()
    for text in (agent, stack):
        assert "@" not in text.replace("@racer", "")  # no unreplaced @TOKEN@
        assert f"WorkingDirectory={repo}" in text
        assert f"User={getpass.getuser()}" in text
        assert "SupplementaryGroups=docker" in text
        assert f"RACER_APP_STATE_DIR={sb.state}" in text
    assert f"ExecStart={repo}/tools/racer_app/agent/.venv/bin/racer-agent serve" in agent
    assert "ExecStart=" + repo + "/tools/racer_app/agent/.venv/bin/racer-agent autostart" in stack
    assert f"ConditionPathExists={sb.state}/autostart.enabled" in stack
    assert "Type=oneshot" in stack and "Restart=always" in agent


def test_stack_unit_never_arms_or_drives(sb):
    sb.run(INSTALL)
    stack = (sb.etc / "systemd/system/racer-stack.service").read_text()
    exec_lines = [line for line in stack.splitlines() if line.startswith("Exec")]
    assert exec_lines == [
        f"ExecStart={DEPLOY.parents[2]}/tools/racer_app/agent/.venv/bin/racer-agent autostart"
    ]


def test_sudoers_allows_exactly_reboot_and_poweroff(sb):
    sb.run(INSTALL)
    path = sb.etc / "sudoers.d/racer-app"
    lines = [x for x in path.read_text().splitlines() if x and not x.startswith("#")]
    assert lines == [
        (
            f"{getpass.getuser()} ALL=(root) NOPASSWD: /usr/bin/systemctl reboot, "
            "/usr/bin/systemctl poweroff"
        )
    ]
    assert stat.S_IMODE(path.stat().st_mode) == 0o440


def test_avahi_file_advertises_racer_and_http(sb):
    sb.run(INSTALL)
    root = ET.parse(sb.etc / "avahi/services/racer.service").getroot()
    services = {s.findtext("type"): s.findtext("port") for s in root.findall("service")}
    assert services == {"_racer._tcp": "8080", "_http._tcp": "8080"}


def test_avahi_follows_a_configured_port(sb):
    sb.run(INSTALL)
    raw = json.loads(sb.config.read_text())
    raw["port"] = 9090
    sb.config.write_text(json.dumps(raw))
    sb.run(INSTALL)
    root = ET.parse(sb.etc / "avahi/services/racer.service").getroot()
    assert {s.findtext("port") for s in root.findall("service")} == {"9090"}


def test_services_are_enabled_and_the_agent_restarted_but_the_heartbeat_is_untouched(sb):
    sb.run(INSTALL)
    calls = sb.systemctl_calls()
    assert "daemon-reload" in calls
    assert "enable racer-agent.service racer-stack.service" in calls
    assert "restart racer-agent.service" in calls
    assert not any("heartbeat" in c for c in calls)
    assert not any("racer-stack.service" in c and c.startswith(("start", "restart")) for c in calls)


def test_second_install_is_idempotent_and_does_not_reprint_or_change_the_token(sb):
    first = sb.run(INSTALL)
    token_hash = json.loads(sb.config.read_text())["token_sha256"]
    second = sb.run(INSTALL)
    assert second.returncode == 0, second.stderr
    assert "RACER APP TOKEN" not in second.stdout and token_in(first.stdout) not in second.stdout
    assert "Token: unchanged" in second.stdout
    assert json.loads(sb.config.read_text())["token_sha256"] == token_hash


def test_an_opt_out_of_autostart_survives_a_reinstall(sb):
    sb.run(INSTALL)
    (sb.state / "autostart.enabled").unlink()  # what the UI toggle does
    r = sb.run(INSTALL)
    assert not (sb.state / "autostart.enabled").exists()
    assert "Autostart: OFF" in r.stdout


def test_no_autostart_flag_on_a_fresh_install(sb):
    r = sb.run(INSTALL, "--no-autostart")
    assert not (sb.state / "autostart.enabled").exists() and "Autostart: OFF" in r.stdout


def test_unknown_arguments_are_rejected(sb):
    assert sb.run(INSTALL, "--bogus").returncode == 2
    assert sb.run(UNINSTALL, "--bogus").returncode == 2


def test_uninstall_removes_the_app_but_keeps_config_and_never_touches_the_heartbeat(sb):
    sb.run(INSTALL)
    sb.systemctl_log.unlink()
    r = sb.run(UNINSTALL)
    assert r.returncode == 0, r.stderr
    for gone in (
        "systemd/system/racer-agent.service",
        "systemd/system/racer-stack.service",
        "sudoers.d/racer-app",
        "avahi/services/racer.service",
    ):
        assert not (sb.etc / gone).exists()
    assert sb.config.exists() and sb.state.exists()
    assert not any("heartbeat" in c for c in sb.systemctl_calls())
    assert "racer-heartbeat.service" in r.stdout  # said out loud that it was left alone


def test_uninstall_purge_and_second_uninstall(sb):
    sb.run(INSTALL)
    assert sb.run(UNINSTALL, "--purge").returncode == 0
    assert not sb.config.exists() and not sb.state.exists()
    assert sb.run(UNINSTALL).returncode == 0  # nothing left: still fine


def test_reinstall_after_uninstall_gets_a_fresh_token_only_after_purge(sb):
    first = sb.run(INSTALL)
    sb.run(UNINSTALL)
    kept = sb.run(INSTALL)
    assert "RACER APP TOKEN" not in kept.stdout
    sb.run(UNINSTALL, "--purge")
    fresh = sb.run(INSTALL)
    assert "RACER APP TOKEN" in fresh.stdout
    assert token_in(fresh.stdout) != token_in(first.stdout)
