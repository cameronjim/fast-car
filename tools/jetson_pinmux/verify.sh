#!/usr/bin/env bash
# Verify, after a reboot, that a 40-pin header pad is really configured as a driven
# GPIO output, then hold it high long enough for a human to meter the physical pin.
#
#   sudo ./verify.sh <gpiochip0-line> [seconds]
#
#   sudo ./verify.sh 144 15    # header pin 7  -- PAC.06, pad soc_gpio59_pac6
#   sudo ./verify.sh  41 15    # header pin 32 -- PG.06,  pad soc_gpio19_pg6
#
# Run it on the Jetson. Needs root to read the pinctrl debugfs. It DOES drive a
# GPIO line, so stop anything else using that pin first, e.g.:
#   sudo systemctl stop racer-heartbeat
#
# The two debugfs files, and what to read from each:
#   pinmux-pins   -- ownership. "(MUX UNCLAIMED)" means no pinctrl node claimed
#                    this pad. A pad configured by this overlay shows "(HOG)
#                    function <f> group <pad>".
#   pinconf-groups -- the live pad config. THIS is where tristate lives. On Tegra,
#                    pinconf-PINS prints nothing but the pin name (the driver's
#                    per-pin dbg_show is an empty function), so do not use it.
#                    tristate=0 means the output driver is enabled.
set -euo pipefail

LINE="${1:-}"
SECS="${2:-15}"
CHIP="${CHIP:-gpiochip0}"
PINMUX_DBG="${PINMUX_DBG:-/sys/kernel/debug/pinctrl/2430000.pinmux}"

die() { echo "verify.sh: $*" >&2; exit 1; }
[ -n "$LINE" ] || die "usage: sudo ./verify.sh <gpiochip0-line> [seconds]"
[ "$(id -u)" -eq 0 ] || die "must be run as root (the pinctrl debugfs is root-only)"
command -v gpioset >/dev/null 2>&1 || die "gpioset not found; sudo apt-get install -y gpiod"
[ -d "$PINMUX_DBG" ] || die "no $PINMUX_DBG -- is debugfs mounted, and is that the right pinctrl node?"

# ---- 1. what does the kernel call this line?
echo "==> gpioinfo $CHIP line $LINE"
gpioinfo "$CHIP" 2>/dev/null | awk -v l="$LINE" '$1=="line" && $2==(l":")' || true
LINE_NAME="$(gpioinfo "$CHIP" 2>/dev/null | awk -v l="$LINE" '$1=="line" && $2==(l":") {print $3}' | tr -d '":' || true)"
[ -n "$LINE_NAME" ] || die "no such line $LINE on $CHIP"
echo "    port name: $LINE_NAME"

# Map the libgpiod port name (PAC.06) onto the pinctrl pad name (soc_gpio59_pac6)
# by matching the pad-name suffix, so this script carries no lookup table of its own.
SUFFIX="$(echo "$LINE_NAME" | tr 'A-Z' 'a-z' | tr -d '.')"
PAD="$(grep -oE "[a-z0-9_]+_${SUFFIX}\b" "${PINMUX_DBG}/pinmux-pins" | head -n1 || true)"
[ -n "$PAD" ] || die "could not map port $LINE_NAME to a pinctrl pad name; grep ${PINMUX_DBG}/pinmux-pins by hand"
echo "    pinctrl pad: $PAD"

# ---- 2. ownership
echo
echo "==> ${PINMUX_DBG}/pinmux-pins"
grep -E "\(${PAD^^}\)" "${PINMUX_DBG}/pinmux-pins" || grep -i "$PAD" "${PINMUX_DBG}/pinmux-pins" || echo "    no line for $PAD"

# ---- 3. the pad config that actually matters
echo
echo "==> ${PINMUX_DBG}/pinconf-groups  [$PAD]"
CONF="$(awk -v p="($PAD):" '$0 ~ p {f=1; print; next} f && /^\t/ {print; next} f {exit}' "${PINMUX_DBG}/pinconf-groups")"
echo "$CONF"

TRI="$(echo "$CONF" | awk -F= '/tristate=/{print $2}' | tr -d ' \t')"
echo
if [ "$TRI" = "0" ]; then
    echo "    tristate=0 -- output driver ENABLED. The overlay is in effect for this pad."
elif [ "$TRI" = "1" ]; then
    echo "    tristate=1 -- output driver DISABLED. The pad will read 0.0 V no matter what"
    echo "    the GPIO controller says. The overlay is NOT in effect for this pad:"
    echo "      * is this pad listed in racer-hdr40-gpio.dts?"
    echo "      * did you reboot after install.sh?"
    echo "      * does the OVERLAYS line in /boot/extlinux/extlinux.conf name"
    echo "        /boot/racer-hdr40-gpio.dtbo, in the LABEL the DEFAULT line selects?"
    echo "    Note: overlays are merged into the LIVE fdt, not into"
    echo "    /sys/firmware/devicetree/base -- check with 'fdtdump /sys/firmware/fdt',"
    echo "    not by looking for the node under /proc/device-tree."
else
    echo "    could not parse tristate from pinconf-groups; read the block above by hand."
fi

# ---- 4. drive it and let a human meter it
echo
echo "==> driving $CHIP line $LINE ($LINE_NAME, header pad $PAD) HIGH for ${SECS}s"
echo "    METER THE HEADER PIN NOW. Black lead on pin 9 (GND). Expect about 3.3 V."
echo "    Sanity-check the meter on pin 17 first: that is the 3.3 V rail."
gpioset --mode=time --sec="$SECS" "$CHIP" "${LINE}=1"
echo "==> released; the line returns to its default state."
