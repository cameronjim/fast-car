#!/usr/bin/env bash
# Undo install.sh: restore the most recent extlinux.conf backup it made.
#
# Run ON THE JETSON, as root:   sudo ./rollback.sh
#
# The .dtbo itself is left in /boot (an unreferenced .dtbo does nothing). Pass
# --remove-dtbo to delete it too.
set -euo pipefail

EXTLINUX="/boot/extlinux/extlinux.conf"
DTBO_DEST="/boot/racer-hdr40-gpio.dtbo"
REMOVE_DTBO=0
[ "${1:-}" = "--remove-dtbo" ] && REMOVE_DTBO=1

die() { echo "rollback.sh: $*" >&2; exit 1; }
[ "$(id -u)" -eq 0 ] || die "must be run as root (sudo ./rollback.sh)"

LATEST="$(ls -1 "${EXTLINUX}".racer-pinmux-backup.* 2>/dev/null | sort | tail -n 1 || true)"
[ -n "$LATEST" ] || die "no backup found matching ${EXTLINUX}.racer-pinmux-backup.*"

echo "==> restoring $LATEST -> $EXTLINUX"
diff -u "$EXTLINUX" "$LATEST" || true
cp -a "$LATEST" "$EXTLINUX"

if [ "$REMOVE_DTBO" -eq 1 ] && [ -f "$DTBO_DEST" ]; then
    rm -f "$DTBO_DEST"
    echo "==> removed $DTBO_DEST"
fi

echo
echo "Restored. Reboot for it to take effect:  sudo reboot"
