#!/usr/bin/env bash
# Remove the racer app from the Jetson. Idempotent.
#
#     tools/racer_app/deploy/uninstall.sh [--purge]
#
# Stops and removes racer-agent.service, racer-stack.service, the sudoers drop-in and the avahi
# service file. It does NOT touch racer-heartbeat.service, the safety mux, or any container
# (a running car-stack keeps running and keeps recording). With --purge it also deletes the
# config (token) and the state directory (action log, autostart choice).
set -euo pipefail

SUDO="${RACER_SUDO-sudo}"
ETC="${RACER_ETC:-/etc}"
STATE_DIR="${RACER_STATE_DIR:-/var/lib/racer-app}"
SYSTEMCTL="${RACER_SYSTEMCTL:-${SUDO:+$SUDO }systemctl}"
PURGE=0
for arg in "$@"; do
  case "$arg" in
    --purge) PURGE=1 ;;
    -h | --help) sed -n '2,11p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

say() { printf '==> %s\n' "$*"; }

say "stopping and disabling the services"
$SYSTEMCTL disable --now racer-agent.service > /dev/null 2>&1 || true
$SYSTEMCTL disable racer-stack.service > /dev/null 2>&1 || true

say "removing unit, sudoers and avahi files"
$SUDO rm -f "$ETC/systemd/system/racer-agent.service" "$ETC/systemd/system/racer-stack.service" \
  "$ETC/sudoers.d/racer-app" "$ETC/avahi/services/racer.service"
$SYSTEMCTL daemon-reload || true
$SYSTEMCTL restart avahi-daemon.service > /dev/null 2>&1 || true

if [ "$PURGE" -eq 1 ]; then
  say "purging config and state"
  $SUDO rm -rf "$ETC/racer-app" "$STATE_DIR"
else
  say "kept $ETC/racer-app (token hash) and $STATE_DIR (action log); --purge removes them"
fi

echo
echo "Left alone on purpose: racer-heartbeat.service, the car-stack and car-camera containers."
echo "Stop a running stack with: docker kill -s INT car-stack"
