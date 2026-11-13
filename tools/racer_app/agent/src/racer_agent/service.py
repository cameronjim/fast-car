"""The HTTP API and the static web UI.

One route, one whitelisted `Controller` method. No route takes a command, a path or a shell
string; unknown fields in a body are rejected. Auth is a bearer token (its hash is in the
config file). The agent also refuses clients outside private address ranges ("LAN only").

`POST /api/stop` (STOP ALL) deliberately needs no token: it can only make the car quieter, and
a phone whose token is stale must still be able to stop it. It is still LAN-only and logged.
That is a software stop. Wi-Fi can drop; the RC kill knob is the stop that always works.
"""

from __future__ import annotations

import ipaddress
import socket
import threading
import time
from collections import defaultdict, deque
from collections.abc import Callable
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import APIRouter, Depends, FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict

from . import __version__
from .commands import StackFlags
from .config import Settings, token_matches
from .controller import ActionError, Controller
from .diskguard import DiskGuard
from .rosstate import FoxgloveClient

FAIL_LIMIT = 10
FAIL_WINDOW_S = 60.0


class Body(BaseModel):
    model_config = ConfigDict(extra="forbid")


class StackBody(Body):
    lidar: bool = True
    vesc: bool = True
    mapping: bool = False
    localization: bool = False
    map: str = ""


class RestartBody(Body):
    flags: StackBody | None = None


class DriveBody(Body):
    mode: str
    profile: str = ""
    raceline: str = ""
    policy: str = ""
    override: bool = False
    override_reason: str = ""
    patrol: str = ""
    rule: str = "arms_raised"
    speed_mps: float = 0.5
    pnp_perception: str = ""
    pnp_dynamics: str = ""
    pnp_backend: str = "net"
    pnp_v_max_mps: float = 0.5
    pnp_params: str = ""
    max_speed_mps: float | None = None
    min_speed_mps: float | None = None


class PerceptionBody(Body):
    mode: str = "pose"
    device: str = "cuda"


class ParkBody(Body):
    override: bool = False
    override_reason: str = ""


class MapBody(Body):
    name: str


class ConfirmBody(Body):
    confirm: str


class BagBody(Body):
    name: str
    confirm: str


class SettingsBody(Body):
    autostart: bool


class AuthGate:
    """Bearer token check with a per-IP lockout so the token cannot be guessed on the LAN."""

    def __init__(self, settings: Settings, clock: Callable[[], float] = time.monotonic) -> None:
        self.settings = settings
        self._clock = clock
        self._fails: dict[str, deque[float]] = defaultdict(deque)
        self._lock = threading.Lock()

    def _recent(self, ip: str) -> deque[float]:
        dq = self._fails[ip]
        now = self._clock()
        while dq and now - dq[0] > FAIL_WINDOW_S:
            dq.popleft()
        return dq

    def check(self, request: Request) -> None:
        ip = client_ip(request)
        with self._lock:
            if len(self._recent(ip)) >= FAIL_LIMIT:
                raise ActionError("too many failed attempts, wait a minute", 429)
        header = request.headers.get("authorization", "")
        presented = header[7:].strip() if header.lower().startswith("bearer ") else None
        if not token_matches(self.settings, presented):
            with self._lock:
                self._recent(ip).append(self._clock())
            raise ActionError("missing or wrong token", 401)


def client_ip(request: Request) -> str:
    return request.client.host if request.client else "unknown"


def is_lan(host: str) -> bool:
    try:
        addr = ipaddress.ip_address(host.split("%")[0])
    except ValueError:
        return False
    if addr.version == 6 and addr.ipv4_mapped is not None:
        addr = addr.ipv4_mapped
    return addr.is_private or addr.is_loopback or addr.is_link_local


_NO_UI = """<!doctype html><meta charset=utf-8><title>Racer</title>
<body style="font-family:sans-serif;max-width:40em;margin:2em auto;padding:0 1em">
<h1>Racer agent is running</h1>
<p>The web UI has not been built on this machine. Build it
(<code>tools/racer_app/deploy/install.sh</code> does) or copy a built
<code>web/dist</code> into <code>tools/racer_app/web/dist</code>.</p>
<p>The API is up: <a href="/health">/health</a>.</p></body>"""


def create_app(
    controller: Controller,
    settings: Settings,
    web_dist: Path | None = None,
    start_background: bool = True,
    lan_only: bool = True,
) -> FastAPI:
    auth = AuthGate(settings)
    foxglove = FoxgloveClient(settings.foxglove_url, controller.ros)
    guard = DiskGuard(controller)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        if start_background:
            controller.startup_check()
            foxglove.start_thread()
            guard.start()
        yield
        if start_background:
            guard.stop()
            foxglove.stop_thread()
            controller.shutdown_mark()

    app = FastAPI(
        title="racer-agent", docs_url=None, redoc_url=None, openapi_url=None, lifespan=lifespan
    )
    app.state.controller = controller

    @app.middleware("http")
    async def lan_and_headers(request: Request, call_next):
        if lan_only and not is_lan(client_ip(request)):
            return JSONResponse({"ok": False, "error": "LAN only"}, status_code=403)
        response = await call_next(request)
        if request.url.path.startswith("/api") or request.url.path == "/health":
            response.headers["Cache-Control"] = "no-store"
        return response

    @app.exception_handler(ActionError)
    async def action_error_handler(_: Request, exc: ActionError):
        return JSONResponse({"ok": False, "error": exc.message, **exc.data}, status_code=exc.status)

    def need_auth(request: Request) -> str:
        auth.check(request)
        return client_ip(request)

    def ok(result: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, **result}

    @app.get("/health")
    def health() -> dict[str, Any]:
        return {
            "ok": True,
            "name": "racer-agent",
            "version": __version__,
            "hostname": socket.gethostname(),
        }

    api = APIRouter(prefix="/api")
    c = controller

    # -- STOP ALL: no token, see the module docstring -------------------------------------
    @api.post("/stop")
    def stop(request: Request) -> dict[str, Any]:
        return ok(c.stop_all(actor=client_ip(request)))

    guarded = APIRouter(prefix="/api", dependencies=[Depends(need_auth)])

    @guarded.get("/status")
    def status() -> dict[str, Any]:
        return c.status()

    @guarded.get("/options")
    def options() -> dict[str, Any]:
        return c.options()

    @guarded.get("/log")
    def action_log(limit: int = 100) -> dict[str, Any]:
        return {"entries": c.actions.tail(max(1, min(limit, 500)))}

    @guarded.get("/vehicle")
    def vehicle() -> dict[str, Any]:
        return c.vehicle()

    @guarded.get("/bags")
    def bags() -> dict[str, Any]:
        return {"bags": c.list_bags(), "disk_free_gb": c.host.disk_free_gb(settings.bags)}

    @guarded.get("/drive/log")
    def drive_log() -> dict[str, Any]:
        return {"log": c.drive_log()}

    @guarded.post("/stack/start")
    def stack_start(body: StackBody, actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.start_stack(StackFlags(**body.model_dump()), actor))

    @guarded.post("/stack/stop")
    def stack_stop(actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.stop_stack(actor))

    @guarded.post("/stack/restart")
    def stack_restart(body: RestartBody, actor: str = Depends(need_auth)) -> dict[str, Any]:
        flags = StackFlags(**body.flags.model_dump()) if body.flags else None
        return ok(c.restart_stack(flags, actor))

    @guarded.post("/cameras/start")
    def cameras_start(actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.start_cameras(actor))

    @guarded.post("/cameras/stop")
    def cameras_stop(actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.stop_cameras(actor))

    @guarded.post("/perception/start")
    def perception_start(body: PerceptionBody, actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.start_perception(body.mode, body.device, actor))

    @guarded.post("/perception/stop")
    def perception_stop(actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.stop_perception(actor))

    @guarded.post("/drive/start")
    def drive_start(body: DriveBody, actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.start_drive(actor=actor, **body.model_dump()))

    @guarded.post("/park/{name}")
    def park(name: str, body: ParkBody, actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.park_command(name, body.override, body.override_reason, actor))

    @guarded.post("/map/save")
    def map_save(body: MapBody, actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.save_map(body.name, actor))

    @guarded.post("/bags/delete")
    def bag_delete(body: BagBody, actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.delete_bag(body.name, body.confirm, actor))

    @guarded.post("/system/reboot")
    def reboot(body: ConfirmBody, actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.power("reboot", body.confirm, actor))

    @guarded.post("/system/shutdown")
    def shutdown(body: ConfirmBody, actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.power("shutdown", body.confirm, actor))

    @guarded.post("/settings")
    def settings_post(body: SettingsBody, actor: str = Depends(need_auth)) -> dict[str, Any]:
        return ok(c.set_autostart(body.autostart, actor))

    app.include_router(api)
    app.include_router(guarded)

    dist = web_dist
    if dist is not None and (dist / "index.html").is_file():
        app.mount("/", StaticFiles(directory=dist, html=True), name="ui")
    else:

        @app.get("/", response_class=HTMLResponse)
        def no_ui() -> str:
            return _NO_UI

    return app
