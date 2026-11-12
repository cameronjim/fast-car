#!/usr/bin/env bash
# Install the racer app on the Jetson. Run as the normal user (racer), from the repo checkout:
#
#     tools/racer_app/deploy/install.sh [--no-autostart]
#
# Idempotent: run it again after every `git pull`. It never regenerates the access token (that
# is only printed the first time) and never flips the autostart choice you made in the web UI.
# It does not touch racer-heartbeat.service, the safety mux, or any running container.
#
# What it does:
#   1. builds the agent's Python environment (uv, from the committed lockfile) and the web UI
#   2. creates /etc/racer-app/config.json (token hash only) and /var/lib/racer-app
#   3. installs racer-agent.service (web UI + API on port 8080) and racer-stack.service
#      (autostart, ON by default on a first install, opt out in the UI or with --no-autostart)
#   4. a sudoers drop-in allowing exactly `systemctl reboot` and `systemctl poweroff`
#   5. an avahi service file advertising _racer._tcp and _http._tcp
#
# Test hooks (used by the agent's pytest, not for normal use): RACER_SUDO, RACER_ETC,
# RACER_STATE_DIR, RACER_SYSTEMCTL, RACER_USER, RACER_AGENT_BIN, RACER_SKIP_BUILD.
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$APP_DIR/../.." && pwd)"
SUDO="${RACER_SUDO-sudo}"
ETC="${RACER_ETC:-/etc}"
STATE_DIR="${RACER_STATE_DIR:-/var/lib/racer-app}"
SYSTEMCTL="${RACER_SYSTEMCTL:-${SUDO:+$SUDO }systemctl}"
USER_NAME="${RACER_USER:-$(id -un)}"
GROUP_NAME="$(id -gn "$USER_NAME")"
AGENT_BIN="${RACER_AGENT_BIN:-$APP_DIR/agent/.venv/bin/racer-agent}"
CONFIG="$ETC/racer-app/config.json"
UV_VERSION="0.12.5"
AUTOSTART_DEFAULT=1

for arg in "$@"; do
  case "$arg" in
    --no-autostart) AUTOSTART_DEFAULT=0 ;;
    -h | --help) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

say() { printf '==> %s\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

[ "$(id -u)" -ne 0 ] || [ -n "${RACER_USER:-}" ] || die "run as the normal user (racer), not root; it uses sudo where it must"
[ -f "$APP_DIR/agent/pyproject.toml" ] || die "cannot find $APP_DIR/agent: run this from a repo checkout"

# -- 0. prerequisites -------------------------------------------------------------------------
if [ -z "${RACER_SKIP_BUILD:-}" ]; then
  command -v docker > /dev/null || die "docker is not installed"
  id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx docker \
    || die "$USER_NAME is not in the docker group (sudo usermod -aG docker $USER_NAME, then log in again)"
  command -v python3 > /dev/null || die "python3 is missing"
  python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' || die "python3 >= 3.10 is required"
  if ! command -v uv > /dev/null; then
    say "installing uv $UV_VERSION (pinned) for the agent's locked environment"
    curl -LsSf "https://astral.sh/uv/$UV_VERSION/install.sh" | sh
    export PATH="$HOME/.local/bin:$PATH"
  fi
fi

# -- 1. build ---------------------------------------------------------------------------------
if [ -z "${RACER_SKIP_BUILD:-}" ]; then
  say "building the agent environment from uv.lock"
  (cd "$APP_DIR/agent" && UV_PYTHON_DOWNLOADS=never uv sync --frozen --no-dev)

  # No npm needed on the host: build-web.sh uses a pinned node container, keeps node_modules
  # between runs, and replaces dist/ only after a successful build (see its header).
  "$APP_DIR/deploy/build-web.sh"
  [ -f "$APP_DIR/web/dist/index.html" ] || die "web UI build produced no dist/index.html"
fi

# -- 2. config, token, state ------------------------------------------------------------------
say "config and state directories"
$SUDO install -d -m 0755 -o "$USER_NAME" -g "$GROUP_NAME" "$ETC/racer-app" "$STATE_DIR"
TOKEN_OUT="$(RACER_APP_CONFIG="$CONFIG" "$AGENT_BIN" init)"
$SUDO chown "$USER_NAME:$GROUP_NAME" "$CONFIG"
$SUDO chmod 0600 "$CONFIG"

# Autostart is ON after the first install, and never changed by a later install: the marker
# records that the choice has been made once.
if [ ! -e "$STATE_DIR/autostart.decided" ]; then
  if [ "$AUTOSTART_DEFAULT" -eq 1 ]; then touch "$STATE_DIR/autostart.enabled"; fi
  touch "$STATE_DIR/autostart.decided"
fi

PORT="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("port", 8080))' "$CONFIG")"

# -- 3. units ---------------------------------------------------------------------------------
render() {  # render <template> <destination>
  sed -e "s|@REPO@|$REPO|g" -e "s|@USER@|$USER_NAME|g" -e "s|@GROUP@|$GROUP_NAME|g" -e "s|@ETC@|$ETC|g" \
    -e "s|@STATE_DIR@|$STATE_DIR|g" -e "s|@PORT@|$PORT|g" "$1" | $SUDO tee "$2" > /dev/null
}

say "systemd units"
$SUDO install -d "$ETC/systemd/system"
render "$APP_DIR/deploy/racer-agent.service.in" "$ETC/systemd/system/racer-agent.service"
render "$APP_DIR/deploy/racer-stack.service.in" "$ETC/systemd/system/racer-stack.service"

# -- 4. sudoers -------------------------------------------------------------------------------
say "sudoers drop-in (reboot and poweroff only)"
$SUDO install -d -m 0750 "$ETC/sudoers.d"
SUDOERS_TMP="$(mktemp)"
sed -e "s|@USER@|$USER_NAME|g" "$APP_DIR/deploy/racer-sudoers.in" > "$SUDOERS_TMP"
if command -v visudo > /dev/null; then
  visudo -cf "$SUDOERS_TMP" > /dev/null || { rm -f "$SUDOERS_TMP"; die "generated sudoers file is invalid"; }
fi
$SUDO install -m 0440 "$SUDOERS_TMP" "$ETC/sudoers.d/racer-app"
rm -f "$SUDOERS_TMP"

# -- 5. mDNS ----------------------------------------------------------------------------------
say "avahi service (_racer._tcp and _http._tcp on port $PORT)"
$SUDO install -d "$ETC/avahi/services"
render "$APP_DIR/deploy/racer-avahi.service.in" "$ETC/avahi/services/racer.service"

# -- 6. start ---------------------------------------------------------------------------------
say "enabling services"
$SYSTEMCTL daemon-reload
$SYSTEMCTL enable avahi-daemon.service > /dev/null 2>&1 || true
$SYSTEMCTL restart avahi-daemon.service > /dev/null 2>&1 || true
$SYSTEMCTL enable racer-agent.service racer-stack.service
$SYSTEMCTL restart racer-agent.service

say "done"
echo
echo "  Web UI:   http://$(hostname).local:$PORT/   (or: python3 tools/racer_app/find_car.py on the Mac)"
if [ -e "$STATE_DIR/autostart.enabled" ]; then
  echo "  Autostart: ON (the stack starts at boot; turn it off in the UI under System)"
else
  echo "  Autostart: OFF"
fi
echo "  Logs:     journalctl -u racer-agent -f"
echo
if printf '%s' "$TOKEN_OUT" | grep -q 'RACER APP TOKEN'; then
  echo "$TOKEN_OUT"
  echo "  Save that token now. It is not stored anywhere in plain text and will not be shown again."
else
  echo "  Token: unchanged (shown only on the first install; racer-agent init --rotate-token makes a new one)."
fi
