"""racer-agent command line: serve, init, autostart.

racer-agent init [--rotate-token]   create the config and print the token ONCE
racer-agent serve                   the HTTP API and web UI (racer-agent.service)
racer-agent autostart               start the stack at boot (racer-stack.service)
racer-agent stop-stack              SIGINT the stack and wait for the bag to close (racer-stack.service)
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

from .actionlog import ActionLog
from .config import Settings, config_path, load_settings, write_token
from .controller import ActionError, Controller
from .hostinfo import HostInfo
from .rosstate import RosState
from .runner import Runner


def build_controller(settings: Settings) -> Controller:
    return Controller(
        settings=settings,
        runner=Runner(),
        host=HostInfo(),
        ros=RosState(),
        actions=ActionLog(settings.action_log_path),
    )


def web_dist(settings: Settings) -> Path:
    override = os.environ.get("RACER_WEB_DIST")
    if override:
        return Path(override)
    return settings.repo_dir / "tools" / "racer_app" / "web" / "dist"


def cmd_init(args: argparse.Namespace) -> int:
    path = config_path()
    token = write_token(path, rotate=args.rotate_token)
    if token is None:
        print(f"{path} already has a token; left unchanged (use --rotate-token to replace it).")
        return 0
    print(f"Wrote {path} (mode 600). Only the hash of the token is stored.")
    print()
    print("  RACER APP TOKEN (shown once, save it now):")
    print(f"  {token}")
    print()
    return 0


def cmd_serve(_: argparse.Namespace) -> int:
    import uvicorn

    from .service import create_app

    settings = load_settings()
    if not settings.token_sha256:
        print("No token configured. Run: racer-agent init", file=sys.stderr)
        return 2
    controller = build_controller(settings)
    app = create_app(controller, settings, web_dist=web_dist(settings))
    uvicorn.run(app, host=settings.host, port=settings.port, log_level="info")
    return 0


def cmd_autostart(_: argparse.Namespace) -> int:
    settings = load_settings()
    controller = build_controller(settings)
    try:
        result = controller.autostart_boot()
    except ActionError as exc:
        print(f"autostart refused: {exc.message}", file=sys.stderr)
        return 1
    print(result["message"])
    return 0


def cmd_stop_stack(_: argparse.Namespace) -> int:
    """What racer-stack.service runs on stop: SIGINT the stack and wait for the bag to close.

    Without it a shutdown that docker handles (SIGTERM, then SIGKILL after 10 s) leaves a bag
    with no footer. The wait is the agent's own (stop_timeout_stack_s), so systemd's
    TimeoutStopSec in the unit has to be longer than that.
    """
    settings = load_settings()
    controller = build_controller(settings)
    result = controller.stop_stack(actor="systemd")
    print(result["message"])
    return 0 if not result.get("bag") or result.get("bag_finalized") else 1


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    parser = argparse.ArgumentParser(prog="racer-agent", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p_init = sub.add_parser("init", help="create the config file and a token")
    p_init.add_argument("--rotate-token", action="store_true")
    p_init.set_defaults(func=cmd_init)
    sub.add_parser("serve", help="run the API and web UI").set_defaults(func=cmd_serve)
    sub.add_parser("autostart", help="boot-time stack start").set_defaults(func=cmd_autostart)
    sub.add_parser(
        "stop-stack", help="SIGINT the stack and wait for the bag to close"
    ).set_defaults(func=cmd_stop_stack)
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
