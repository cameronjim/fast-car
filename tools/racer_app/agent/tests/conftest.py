from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))

from fakes import World, healthy_ros, make_host
from racer_agent.actionlog import ActionLog
from racer_agent.config import Settings, hash_token
from racer_agent.controller import Controller
from racer_agent.rosstate import RosState
from racer_agent.runner import FakeRunner

TOKEN = "test-token-123"


class Env:
    """Everything a controller test needs, with a fake clock so nothing sleeps for real."""

    def __init__(self, tmp_path: Path) -> None:
        self.repo = tmp_path / "car"
        (self.repo / "ros_ws" / "install").mkdir(parents=True)
        (self.repo / "ros_ws" / "install" / "setup.bash").write_text("")
        (self.repo / "config" / "tracks" / "gym_oval").mkdir(parents=True)
        (self.repo / "config" / "tracks" / "gym_oval" / "raceline.csv").write_text("x,y\n")
        self.world = World(self.repo)
        self.host = make_host(tmp_path, self.world)
        self.settings = Settings(
            repo_dir=self.repo,
            state_dir=tmp_path / "state",
            token_sha256=hash_token(TOKEN),
        )
        self.ros = RosState()
        healthy_ros(self.ros)
        self.runner = FakeRunner(responders=[self.world.responder])
        self.actions = ActionLog(self.settings.action_log_path)
        self.sleeps: list[float] = []
        self.deferred: list = []
        self.now = 1_000_000.0
        self.controller = Controller(
            self.settings,
            self.runner,
            self.host,
            self.ros,
            self.actions,
            clock=lambda: self.now,
            sleep=self._sleep,
            defer=lambda delay, fn: self.deferred.append((delay, fn)),
        )

    def _sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += seconds  # defeats the snapshot cache the way real time would

    def running_car(self) -> None:
        """A stack up with a recorder, healthy ROS, ready to drive."""
        self.world.start_stack()

    def last_log(self) -> dict:
        return self.actions.tail(1)[0]


@pytest.fixture
def env(tmp_path: Path) -> Env:
    return Env(tmp_path)
