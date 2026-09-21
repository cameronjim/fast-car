#!/usr/bin/env bash
# Install the racer 40-pin header pinmux overlay on a Jetson Orin Nano Super Dev Kit
# (JetPack 6.2 / L4T R36.4.4).
#
# Run this ON THE JETSON, as root:   sudo ./install.sh
#
# Idempotent: running it twice leaves the same extlinux.conf and the same .dtbo.
# It refuses to touch anything if the .dts does not compile.
#
# NOTHING TAKES EFFECT UNTIL YOU REBOOT. See README.md in this directory.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DTS="${SCRIPT_DIR}/racer-hdr40-gpio.dts"
DTBO_NAME="racer-hdr40-gpio.dtbo"
DTBO_DEST="/boot/${DTBO_NAME}"
EXTLINUX="/boot/extlinux/extlinux.conf"

die() { echo "install.sh: $*" >&2; exit 1; }

[ "$(id -u)" -eq 0 ] || die "must be run as root (sudo ./install.sh)"
[ -f "$DTS" ] || die "cannot find $DTS"
[ -f "$EXTLINUX" ] || die "cannot find $EXTLINUX -- is this a Jetson with extlinux boot?"
command -v dtc >/dev/null 2>&1 || die "dtc not found; sudo apt-get install -y device-tree-compiler"

# ---------------------------------------------------------------- compile
# -@ emits __symbols__/__fixups__. This overlay needs them: it targets the base
# DTB's &pinmux and &pinmux_aon by label, exactly as the jetson-io generated
# overlay does, and those are resolved from the fixups at overlay-apply time.
TMP_DTBO="$(mktemp -t racer-hdr40-gpio.XXXXXX)"
trap 'rm -f "$TMP_DTBO"' EXIT
echo "==> compiling $DTS"
dtc -@ -I dts -O dtb -o "$TMP_DTBO" "$DTS" \
    || die "the .dts did not compile -- refusing to install anything"
echo "    ok: $(stat -c %s "$TMP_DTBO") bytes"

# ---------------------------------------------------------------- backup
STAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP="${EXTLINUX}.racer-pinmux-backup.${STAMP}"
cp -a "$EXTLINUX" "$BACKUP"
echo "==> backed up $EXTLINUX -> $BACKUP"

# ---------------------------------------------------------------- install dtbo
install -m 0644 "$TMP_DTBO" "$DTBO_DEST"
echo "==> installed $DTBO_DEST"

# ---------------------------------------------------------------- edit extlinux
# Rules:
#   * Add our .dtbo to every OVERLAYS line, or create one if there is none.
#   * REMOVE /boot/jetson-io-hdr40-user-custom.dtbo if it is listed. This overlay
#     replaces it: both define an exp-header-pinmux node under the same &pinmux
#     target and both set that node as pinctrl-0, so listing both leaves it to
#     merge order which node's hog actually applies. Ours carries the pwm1
#     (pin 15) and pwm5 (pin 33) pad nodes copied verbatim from it, so nothing is
#     lost except pwm7 (pin 32), which is deliberate -- pin 32 is the fallback
#     heartbeat pin. See README.md, "Ordering and coexistence".
#     The jetson-io .dtbo file itself is left in /boot, simply unreferenced.
#   * Never duplicate our path if it is already listed.
echo "==> editing $EXTLINUX"
python3 "${SCRIPT_DIR}/_edit_extlinux.py" "$EXTLINUX" "$DTBO_DEST"

# ---------------------------------------------------------------- diff
echo "==> diff ($BACKUP -> $EXTLINUX)"
diff -u "$BACKUP" "$EXTLINUX" || true

echo
echo "Done. NOTHING IS ACTIVE UNTIL YOU REBOOT."
echo "  sudo reboot"
echo
echo "Then verify (stop anything already using the line first, e.g."
echo "'sudo systemctl stop racer-heartbeat'):"
echo "  sudo ./verify.sh 144 15   # header pin 7,  gpiochip0 line 144 (PAC.06)"
echo "  sudo ./verify.sh  41 15   # header pin 32, gpiochip0 line 41  (PG.06)"
echo
echo "To undo:  sudo ./rollback.sh"
