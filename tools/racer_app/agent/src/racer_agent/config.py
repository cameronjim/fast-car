"""Agent settings, the token, and the thresholds the pre-drive checks use.

The config file is JSON (Python 3.10 on the Jetson has no tomllib). It holds the SHA-256 of
the access token, never the token itself: `racer-agent init` prints the token once and stores
only the hash. Everything else has a default that matches the car as it is set up today.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import os
import secrets
from dataclasses import dataclass, field, fields
from pathlib import Path

DEFAULT_CONFIG_PATH = Path("/etc/racer-app/config.json")
DEFAULT_STATE_DIR = Path("/var/lib/racer-app")


@dataclass(frozen=True)
class Thresholds:
    """Pre-drive check thresholds. Documented in docs/notes/racer-app-2026-11-11.md.

    Battery (VESC input voltage, per cell for a 3S pack): green at or above `cell_green_v`,
    amber down to `cell_red_v`, red below it. 3.70 V per cell is roughly 40 percent charge at
    rest; 3.50 V per cell is the point where a loaded pack sags towards the VESC's own cutoff.
    These are conservative starting values, not a measured curve for this pack.
    """

    battery_cells: int = 3
    cell_green_v: float = 3.70
    cell_red_v: float = 3.50
    cell_max_v: float = 4.25  # above this the cell count setting is probably wrong
    min_plausible_pack_v: float = 6.0  # below this the VESC is on USB or has no pack
    disk_green_gb: float = 15.0
    disk_red_gb: float = 5.0  # below this nothing may start recording
    cameras_min_free_gb: float = 10.0  # cameras write about 6 MB/s per camera
    guard_cameras_gb: float = 7.0  # the disk guard stops the cameras below this
    guard_drive_gb: float = 3.0  # ... stops the driving launch below this
    guard_stack_gb: float = 1.5  # ... and shuts the stack down cleanly below this
    scan_green_hz: float = 5.0  # the RPLIDAR C1 runs at about 10 Hz
    scan_red_hz: float = 1.0
    vesc_max_age_s: float = 1.0  # VESC telemetry older than this is stale
    cpu_amber_c: float = 70.0
    cpu_red_c: float = 85.0
    vesc_amber_c: float = 60.0
    vesc_red_c: float = 80.0
    wifi_amber_dbm: float = -65.0
    wifi_red_dbm: float = -75.0
    uptime_amber_s: float = 120.0  # a car that booted under two minutes ago is worth a look


@dataclass(frozen=True)
class Settings:
    repo_dir: Path = Path.home() / "car"
    bag_dir: Path | None = None  # default: <repo_dir>/data/bags
    map_dir: Path | None = None  # default: <repo_dir>/data/maps
    state_dir: Path = DEFAULT_STATE_DIR
    port: int = 8080
    host: str = "0.0.0.0"
    foxglove_url: str = "ws://127.0.0.1:8765"
    image: str = "car:local"
    perception_image: str = "car:perception"  # the GPU image: INSTALL_TORCH + INSTALL_PERCEPTION
    stack_container: str = "car-stack"
    camera_container: str = "car-camera"
    perception_container: str = "car-perception"
    token_sha256: str = ""
    thresholds: Thresholds = field(default_factory=Thresholds)
    # How long a stop waits for SIGINT to finish before it escalates. The stack closes the bag on
    # SIGINT, and a 6 GB mcap took minutes to finalize; SIGTERM or SIGKILL before that leaves a
    # bag with no footer and no metadata.yaml. The other containers write no bag.
    stop_timeout_stack_s: float = 600.0
    stop_timeout_other_s: float = 120.0
    stop_term_grace_s: float = 20.0  # after SIGTERM, before SIGKILL

    @property
    def bags(self) -> Path:
        return self.bag_dir or self.repo_dir / "data" / "bags"

    @property
    def maps(self) -> Path:
        return self.map_dir or self.repo_dir / "data" / "maps"

    @property
    def racelines_user_dir(self) -> Path:
        return self.repo_dir / "data" / "racelines"

    @property
    def patrols_dir(self) -> Path:
        return self.repo_dir / "data" / "patrols"

    @property
    def perception_cache_dir(self) -> Path:
        return self.repo_dir / "data" / "perception-cache"

    @property
    def policies_dir(self) -> Path:
        return self.repo_dir / "data" / "policies"

    @property
    def pnp_models_dir(self) -> Path:
        return self.repo_dir / "data" / "pnp" / "models"

    @property
    def pnp_dir(self) -> Path:
        return self.repo_dir / "data" / "pnp"

    @property
    def action_log_path(self) -> Path:
        return self.state_dir / "actions.jsonl"

    @property
    def autostart_flag(self) -> Path:
        return self.state_dir / "autostart.enabled"

    @property
    def run_marker(self) -> Path:
        return self.state_dir / "run.marker"


def hash_token(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def token_matches(settings: Settings, presented: str | None) -> bool:
    if not presented or not settings.token_sha256:
        return False
    return hmac.compare_digest(hash_token(presented), settings.token_sha256)


def new_token() -> str:
    return secrets.token_urlsafe(24)


def config_path() -> Path:
    return Path(os.environ.get("RACER_APP_CONFIG", str(DEFAULT_CONFIG_PATH)))


_THRESHOLD_KEYS = {f.name for f in fields(Thresholds)}


def load_settings(path: Path | None = None) -> Settings:
    """Load settings from the JSON file (missing file: defaults and no token)."""
    cfg_path = path or config_path()
    raw: dict = {}
    if cfg_path.exists():
        raw = json.loads(cfg_path.read_text(encoding="utf-8"))
    repo = Path(os.environ.get("RACER_REPO_DIR") or raw.get("repo_dir") or Path.home() / "car")
    state = Path(os.environ.get("RACER_APP_STATE_DIR") or raw.get("state_dir") or DEFAULT_STATE_DIR)
    thresholds = Thresholds(
        **{k: v for k, v in raw.get("thresholds", {}).items() if k in _THRESHOLD_KEYS}
    )
    return Settings(
        repo_dir=repo,
        bag_dir=Path(raw["bag_dir"]) if raw.get("bag_dir") else None,
        map_dir=Path(raw["map_dir"]) if raw.get("map_dir") else None,
        state_dir=state,
        port=int(raw.get("port", 8080)),
        host=str(raw.get("host", "0.0.0.0")),
        foxglove_url=str(raw.get("foxglove_url", "ws://127.0.0.1:8765")),
        image=str(raw.get("image", "car:local")),
        perception_image=str(raw.get("perception_image", "car:perception")),
        token_sha256=str(raw.get("token_sha256", "")),
        thresholds=thresholds,
        stop_timeout_stack_s=float(raw.get("stop_timeout_stack_s", 600.0)),
        stop_timeout_other_s=float(raw.get("stop_timeout_other_s", 120.0)),
        stop_term_grace_s=float(raw.get("stop_term_grace_s", 20.0)),
    )


def write_token(path: Path, rotate: bool = False) -> str | None:
    """Create the config with a fresh token hash. Returns the plaintext token ONCE.

    With an existing token and `rotate=False` nothing is changed and None is returned, so an
    idempotent install never invalidates the token the owner already saved.
    """
    raw: dict = {}
    if path.exists():
        raw = json.loads(path.read_text(encoding="utf-8"))
        if raw.get("token_sha256") and not rotate:
            return None
    token = new_token()
    raw["token_sha256"] = hash_token(token)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        json.dump(raw, fh, indent=2, sort_keys=True)
        fh.write("\n")
    os.replace(tmp, path)
    os.chmod(path, 0o600)
    return token
