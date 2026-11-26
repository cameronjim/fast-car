"""The owner's saved drive settings (the Tune tab), kept on the car so every client sees them.

A JSON file in the agent's state directory. The shape is fixed here and unknown fields are
refused, so the file can never carry a command or a path. The values are only defaults for the
Drive buttons: every drive start is validated again by the controller and the whitelist, and the
nodes enforce their own limits, so a stale or hand-edited file cannot widen anything.

`record_camera` decides whether the camera container publishes its small recording stream for the
stack's bag. It never affects the stack's own recorder: the bag of LiDAR, drive, telemetry and
rail voltage is always recorded (CLAUDE.md invariant 5) and there is no setting for it.
"""

from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, ValidationError

log = logging.getLogger(__name__)


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


class StackPrefs(_Strict):
    mode: Literal["teleop", "mapping", "localization"] = "teleop"
    map: str = ""


class GapFollowPrefs(_Strict):
    profile: str = "lane-2026-10-25"
    # None means "the profile's own value": no launch argument is sent.
    max_speed_mps: float | None = None
    min_speed_mps: float | None = None
    a_lat_max_mps2: float | None = None
    side_cap_min_speed_mps: float | None = None


class PnpPrefs(_Strict):
    v_max_mps: float = 0.5
    backend: str = "geometric"
    dynamics: str = ""
    perception: str = ""
    params_file: str = ""
    # Lap learning (pnp.launch.py 15.7). The defaults send nothing: today's command.
    v_lap1_mps: float | None = None
    lap_memory: bool = False
    lap_mode: Literal["v0", "v1", "observe"] = "v0"
    lap_source: Literal["start_scan", "map_pose", "camera"] = "start_scan"
    lap_counter: bool = True
    require_lap_counter: bool = False
    shadow_perception: str = ""
    # Owner decision 21 ratio length check of the camera lap gate; 0 and 0: the registered band.
    lap_length_ratio_min: float = 0.0
    lap_length_ratio_max: float = 0.0
    # pnp.launch.py lap_rate_hz (the camera lap counter's rate); 0: the launch default (15).
    lap_rate_hz: float = Field(default=0.0, ge=0.0, le=30.0)
    # Prototype switches (owner decision 29): the defaults send nothing.
    dv1_variant: Literal["registered", "K1", "K2", "K3", "K3s", "K4", "K5", "K6"] = "registered"
    dynamics_fast_mlp: bool = False
    dynamics_mlp_float32: bool = False
    driving_reference: bool = False
    dynamics_adaptation: bool = False
    rt_sched: bool = False


class TrackerPrefs(_Strict):
    raceline: str = ""


class Prefs(_Strict):
    record_camera: bool = True
    # cameras.launch.py usb_fps and csi. None and True send nothing: the launch's own defaults.
    camera_usb_fps: int | None = Field(default=None, ge=5, le=60)
    camera_csi: bool = True
    stack: StackPrefs = Field(default_factory=StackPrefs)
    gap_follow: GapFollowPrefs = Field(default_factory=GapFollowPrefs)
    pnp: PnpPrefs = Field(default_factory=PnpPrefs)
    tracker: TrackerPrefs = Field(default_factory=TrackerPrefs)


class PrefsStore:
    def __init__(self, path: Path) -> None:
        self.path = path

    def load(self) -> Prefs:
        """The saved settings, or the defaults when there is no file or it does not parse."""
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
            return Prefs.model_validate(raw)
        except FileNotFoundError:
            return Prefs()
        except (OSError, ValueError, ValidationError) as exc:
            log.warning("ignoring unreadable prefs file %s: %s", self.path, exc)
            return Prefs()

    def save(self, prefs: Prefs) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(prefs.model_dump(), indent=2, sort_keys=True) + "\n", "utf-8")
        os.replace(tmp, self.path)
