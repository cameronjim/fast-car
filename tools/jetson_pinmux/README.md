# tools/jetson_pinmux -- make the Jetson's 40-pin header GPIO pads actually drive

A device tree overlay that turns on the pad output drivers for the 40-pin header pins
this project uses as GPIO outputs, plus install / rollback / verify scripts for it.

**Status: PREPARED, NOT APPLIED.** Nothing in here has been run on the Jetson. The
overlay compiles (checked with `dtc -@` on a host) and its decompiled structure is
byte-for-byte the same shape as the jetson-io overlay currently in use on the board,
but whether the pin measures 3.3 V afterwards is unproven until somebody installs it,
reboots, and meters the pin. Everything below that is marked as measured WAS measured;
everything else is explicitly flagged.

Related: `tools/jetson_heartbeat/` (the thing that is broken by this), `docs/notes/build-log.md`
(2026-09-21 entry), `firmware/safety_mux/` and `docs/notes/mux-diagnostic-build.md`
(the diagnostic firmware that reported the symptom).

## The problem

The Jetson heartbeat that feeds the layer-1 safety mux does not reach the mux. Measured
on the bench Jetson (racer-car, 10.0.0.226, JetPack 6.2 / L4T R36.4.4, Orin Nano Super
Dev Kit, `nvidia,p3768-0000+p3767-0005-super`) on 2026-09-21:

- The mux's diagnostic firmware reports `HB gp5=NO_EDGES_EVER`. No edge has ever
  arrived at the Pico.
- The Jetson kernel toggles `gpiochip0` line 144 (`PAC.06`, pad `soc_gpio59_pac6`,
  header pin 7) perfectly well. `/sys/kernel/debug/gpio` shows `out hi` / `out lo`
  following the commanded value.
- **A multimeter on header pin 7 reads 0.0 V while `gpioset` holds that line high.**
  Header pin 17 reads 3.4 V on the same meter with the same ground lead, so the meter
  and the pin counting are both good.
- Same result on header pins 11, 12, 13, 16, 18, 19 and 21 driven high through
  `gpiochip0` lines 112, 50, 122, 126, 125, 135 and 134: all 0.0 V.
- Header pin 15 (`gpiochip0` line 85, `PN.01`, pad `soc_gpio39_pn1`) also read 0.0 V as
  a GPIO with no overlay. The *same pad* drove a 1.64 V average once a jetson-io overlay
  muxed it to PWM (`pwm1`). Before that overlay existed, PWM on pins 15 and 33 also read
  0.0 V.

So: the kernel is doing its job, and the pad is not.

## The mechanism, as verified

The working hypothesis going in was "unconfigured header pads keep a boot-time pad
config with the output driver tristated". **That hypothesis holds.** It was checked two
ways, and it is not an inference from documentation alone.

### Directly, on this board

`/sys/kernel/debug/pinctrl/2430000.pinmux/pinconf-groups` prints the live pad config.
Read on 2026-09-21 (read-only, nothing was changed):

| header pin | pad | configured by | `tristate` | meter |
|---|---|---|---|---|
| 7  | `soc_gpio59_pac6` | nothing | **1** | 0.0 V |
| 29 | `soc_gpio32_pq5`  | nothing | **1** | 0.0 V |
| 15 | `soc_gpio39_pn1`  | jetson-io `pwm1` | **0** | 1.64 V avg |
| 32 | `soc_gpio19_pg6`  | jetson-io `pwm7` | **0** | (not metered) |
| 33 | `soc_gpio21_ph0`  | jetson-io `pwm5` | **0** | drives |

Every pad that drives has `tristate=0`. Every pad that reads 0.0 V has `tristate=1`.
The full block for pin 7 as read:

```
161 (soc_gpio59_pac6):
	pull=2
	tristate=1
	enable-input=1
	...
	gpio-mode=0
	function=rsvd2
```

and for the working pin 15:

```
5 (soc_gpio39_pn1):
	pull=1
	tristate=0
	enable-input=0
	...
	function=gp
```

`pinmux-pins` on the same read shows 165 of 167 pads as `(MUX UNCLAIMED)`; the three
pads jetson-io configured show up as `(HOG) function gp group <pad>`. Note that
`function gp` here is the **GP PWM controller**, not "general purpose IO".

### Why the two are independent, from the sources

The pad config block and the GPIO controller are different hardware, and nothing in the
GPIO path clears tristate:

- The pad's `TRISTATE` is bit 4 of that pad's PINMUX register in `pinmux@2430000`.
  `drivers/pinctrl/tegra/pinctrl-tegra.c` maps `nvidia,tristate` to
  `TEGRA_PINCONF_PARAM_TRISTATE` with `tri_reg = mux_reg`, `tri_bit = 4`.
- `drivers/gpio/gpio-tegra186.c` drives the line from `gpio@2200000`, through
  `TEGRA186_GPIO_ENABLE_CONFIG` (`_ENABLE` bit 0, `_OUT` bit 1),
  `TEGRA186_GPIO_OUTPUT_CONTROL` (`_FLOATED` bit 0) and `TEGRA186_GPIO_OUTPUT_VALUE`.
  `tegra186_gpio_direction_output()` sets the value, clears `_FLOATED`, and sets
  `_ENABLE | _OUT`. **There is no tristate bit in that register block and the GPIO
  driver never touches the pinmux one.**

So a pad needs all of: pinmux `TRISTATE == 0`, GPIO `OUTPUT_CONTROL.FLOATED == 0`,
`ENABLE_CONFIG.{ENABLE,OUT} == 1`, and `GPIO_SFIO_SEL == 0`. The kernel-level evidence in
`tools/jetson_heartbeat/README.md` covers the last three and says nothing about the
first, which is exactly the gap that let a broken pin look verified.

`GPIO_SFIO_SEL` (bit 10 on these pads) is what picks GPIO over the mux'd special
function. It is not the `nvidia,function` field: `tegra_pinctrl_gpio_request_enable()`
clears it when a GPIO consumer claims the line, and `tegra_pinctrl_set_mux()` sets it
whenever an `nvidia,function` is written. That is why NVIDIA's own pinmux spreadsheets
park GPIO pins on an `rsvdN` function: the mux field just has to be somewhere harmless,
because the GPIO/SFIO choice is made elsewhere. There is a literal `gpio` function
string in `pinctrl-tegra234.c`'s function table, but no pad lists it among its four
selectable functions, so `nvidia,function = "gpio"` cannot be used.

Sources: `drivers/pinctrl/tegra/pinctrl-tegra.c`, `drivers/pinctrl/tegra/pinctrl-tegra234.c`,
`drivers/gpio/gpio-tegra186.c`, `include/dt-bindings/pinctrl/pinctrl-tegra.h`
(`TEGRA_PIN_DISABLE 0` / `TEGRA_PIN_ENABLE 1`),
`Documentation/devicetree/bindings/pinctrl/nvidia,tegra234-pinmux-common.yaml`, and
NVIDIA's [Configuring the Jetson Expansion Headers](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/HR/ConfiguringTheJetsonExpansionHeaders.html)
and [Pinmux and GPIO Configuration](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/SD/Bootloader/PinmuxGpioConfig.html).
The same failure and the same fix are independently reported by
[jetsonhacks/jetson-orin-gpio-patch](https://github.com/jetsonhacks/jetson-orin-gpio-patch)
(whose `pin7_as_gpio.dts` targets this exact pad) and
[thomasthelliez.com](https://thomasthelliez.com/blog/enabling-gpio-output-pins-on-nvidia-jetson-orin-nano-super/).

### Two things not verified

- **Where `tristate=1` comes from.** Almost certainly the MB1 pinmux BCT programmed at
  flash time (`Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-pinmux-*.dtsi`),
  which the kernel then leaves alone for pads no DT node claims. That BCT is not
  mirrored publicly and was not read. It does not matter for the fix: the overlay sets
  tristate explicitly either way.
- **Why pin 7 reads 0.0 V rather than floating high.** The pad reports `pull=2`
  (pull-up) with `pull-up-strength=31`. A tristated pad with a pull-up might have been
  expected to sit near 3.3 V. It measured 0.0 V. Unexplained; possibly the pull is not
  actually enabled in the way the debugfs field suggests, or the GPIO controller drives
  low through a path the meter sees. It does not change the diagnosis or the fix, and
  the overlay sets `nvidia,pull = <0>` on the pads it touches anyway.

## What the overlay does

`racer-hdr40-gpio.dts` configures four pads:

| header pin | pad | `gpiochip0` line | port | as | why |
|---|---|---|---|---|---|
| 7  | `soc_gpio59_pac6` | 144 | `PAC.06` | GPIO output | the heartbeat pin as currently wired |
| 32 | `soc_gpio19_pg6`  | 41  | `PG.06`  | GPIO output | fallback heartbeat pin |
| 15 | `soc_gpio39_pn1`  | 85  | `PN.01`  | PWM (`gp`, `pwm1`) | steering, kept working |
| 33 | `soc_gpio21_ph0`  | 43  | `PH.00`  | PWM (`gp`, `pwm5`) | throttle, kept working |

The two GPIO pads get `nvidia,tristate = <0>` (drive), `nvidia,enable-input = <0>`
(output only), `nvidia,pull = <0>` (no pull), and **no `nvidia,function`**. Omitting the
function means the hog touches pad config only and never writes the mux or
`GPIO_SFIO_SEL`, leaving the GPIO/SFIO choice entirely to the GPIO request path that
libgpiod triggers. That is what the known-working community overlay for pin 7 does, so
it is the lower-risk choice. The legal function names for these pads, read off this
board's `pinmux-functions`, if a future change wants to park the mux explicitly:
`soc_gpio59_pac6` -> `aud`, `i2s8`, `rsvd2`, `rsvd3` (currently `rsvd2`);
`soc_gpio19_pg6` -> `gp`, `rsvd1`, `rsvd2`, `rsvd3` (currently `gp`).

The two PWM nodes are copied verbatim from the decompiled jetson-io overlay.

### Correction to the task's pin 32 assumption

Header pin 32 on this carrier is **`soc_gpio19_pg6` (PG.06, `gpiochip0` line 41)**, not
`gp_pwm2_px2`. `gp_pwm2_px2` is `PX.02`, `gpiochip0` line 116, and it is **not routed to
the 40-pin header at all**. Confirmed two ways: the decompiled jetson-io overlay on this
board emits `hdr40-pin32 { nvidia,pins = "soc_gpio19_pg6"; ... }`, and jetson-io's own
live pinmap (`Jetson.board.Board(...).header.pins.get_name(32)`) returns
`soc_gpio19_pg6`. The overlay uses the correct pad.

### `enable-input = <0>` and reading the pin back

With the input buffer off you cannot `gpioget` or `gpiomon` these two pads. This matters
for one thing: the loopback-jumper test `tools/jetson_heartbeat/README.md` proposes uses
pin 7 as the *driver* and pin 29 as the *reader*, and pin 29 is untouched by this overlay
(`enable-input=1` already), so that test still works. If you ever need to read pin 7 or
pin 32 back, set `nvidia,enable-input = <1>` on that node; tristate only gates the output
driver, so input and driven output can both be on.

### Ordering and coexistence

**This overlay replaces `/boot/jetson-io-hdr40-user-custom.dtbo`. Do not list both.**

Both overlays create a node called `exp-header-pinmux` under the same `&pinmux` target
and both set that node as the pinmux device's `pinctrl-0`. With both listed, which
node's hog actually gets applied comes down to overlay merge order, and the `pinctrl-0`
phandle can only point at one of them. Rather than reason about that, this overlay
carries jetson-io's pwm1 and pwm5 nodes itself, so a single overlay does everything.
`install.sh` removes the jetson-io entry from the `OVERLAYS` line automatically and
prints that it did; the `.dtbo` file stays in `/boot`, just unreferenced, so
`rollback.sh` can put it back.

The one thing lost is **pwm7 on pin 32**, deliberately: pin 32 is the fallback heartbeat
pin and cannot be both a PWM output and a GPIO. Steering (pin 15, pwm1) and throttle
(pin 33, pwm5) are unaffected.

## Install

On the Jetson, from a checkout of this repo:

```sh
cd tools/jetson_pinmux
sudo ./install.sh
sudo reboot
```

`install.sh` compiles the `.dts` and refuses to change anything if it does not compile,
backs up `/boot/extlinux/extlinux.conf` to a timestamped copy, installs
`/boot/racer-hdr40-gpio.dtbo`, rewrites the `OVERLAYS` line, and prints the diff. It is
idempotent.

## Verify (after the reboot)

```sh
sudo systemctl stop racer-heartbeat          # free the line first
cd tools/jetson_pinmux
sudo ./verify.sh 144 15                      # header pin 7,  PAC.06
sudo ./verify.sh  41 15                      # header pin 32, PG.06
```

`verify.sh` prints the pad's `pinmux-pins` and `pinconf-groups` lines, says in plain
words whether `tristate` is 0 or 1, then holds the line high for N seconds so you can
meter the physical pin. Black lead on pin 9 (GND); check the meter on pin 17 (3.3 V
rail) first. **Expect about 3.3 V. 0.0 V means it still is not driving.**

Read `pinconf-groups`, not `pinconf-pins`: the Tegra pinctrl driver's per-pin
`dbg_show` is an empty function, so `pinconf-pins` prints pad names and nothing else.
That is why the earlier debugging saw blank output there.

Also note that an overlay applied through `OVERLAYS` is merged into the **live** FDT,
not into `/sys/firmware/devicetree/base`. Do not conclude the overlay failed because the
node is missing under `/proc/device-tree`; check `fdtdump /sys/firmware/fdt`, or just
read `pinconf-groups`, which reflects the hardware.

Then the end-to-end check, which is the one that actually matters: with the mux's
`DIAG_BUILD` firmware flashed and pin 7 wired to the mux's GP5 with a shared ground,
start `racer-heartbeat` and read the mux's USB CDC port (`cat /dev/ttyACM0`). The
heartbeat field must read `HB gp5 age=<small>ms (timeout 100ms) OK`, not
`HB gp5=NO_EDGES_EVER`.

## Rollback

```sh
sudo ./rollback.sh                 # restore the newest extlinux.conf backup
sudo ./rollback.sh --remove-dtbo   # ...and delete /boot/racer-hdr40-gpio.dtbo
sudo reboot
```

This puts the `OVERLAYS` line back exactly as it was, including the jetson-io entry.

## Re-applying after a JetPack update

A JetPack / L4T update rewrites `/boot/extlinux/extlinux.conf` and may replace the
kernel DTB, so the `OVERLAYS` line disappears and the pads go back to tristated. It can
also change the flashed MB1 pinmux BCT. After any such update:

1. Re-run `sudo ./install.sh` and reboot. It is idempotent, so it is safe to run even
   if the line survived.
2. Re-check the `compatible` list in the `.dts` against `/proc/device-tree/compatible`.
   If the board string is no longer in the list, the overlay is silently skipped. Today
   the board reports `nvidia,p3768-0000+p3767-0005-super`, which is in the list.
3. Re-check header pin 32's pad name: `sudo python3 /opt/nvidia/jetson-io/config-by-pin.py -l`
   and the `get_name()` snippet in `tools/jetson_heartbeat/README.md`. Carrier revisions
   have moved header pins before, which is how the `gp_pwm2_px2` mistake above happened.
4. Re-run the two `verify.sh` commands and re-meter. Never take a green kernel-level
   check as proof again.

Worth knowing: on JP6 the Tegra pinctrl driver's `gpio_disable_free()` unconditionally
re-asserts SFIO instead of restoring the previous value, so a pad can behave differently
after a GPIO consumer releases it. The heartbeat holds its line for the life of the
process, so this does not bite us, but it will confuse manual `gpioset` experiments.

## How to add another GPIO pad

1. Find the pad name for the header pin, on the board, not from a diagram:

   ```sh
   sudo python3 - <<'EOF'
   import sys; sys.path.insert(0, '/opt/nvidia/jetson-io')
   from Jetson import board
   b = board.Board(); b.set_active_header(b.get_board_headers()[0])
   print(b.header.pins.get_name(31))     # <- your header pin number
   EOF
   ```

2. Find its `gpiochip0` line. The pad suffix is the port name: `soc_gpio33_pq6` ->
   `PQ.06` -> `gpioinfo gpiochip0 | grep 'PQ.06'`.

3. Check nothing else owns it:

   ```sh
   sudo grep -i pq6 /sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins
   ```

   `(MUX UNCLAIMED)` is what you want. If it shows `(HOG) function <x>`, something in
   the device tree is already using that pad and you need to work out what first.

4. Add a node to `racer-hdr40-gpio.dts`, keeping the `hdr40-pinNN` naming:

   ```dts
   hdr40-pin31 {
   	nvidia,pins = "soc_gpio33_pq6";
   	nvidia,tristate = <0x00>;
   	nvidia,enable-input = <0x00>;   /* <1> if you also need to read it */
   	nvidia,pull = <0x00>;
   };
   ```

   Leave `nvidia,function` out unless you have a reason. If you add one, it must be one
   of that pad's four legal functions:

   ```sh
   sudo awk '/^function/{f=$0} /soc_gpio33_pq6/{print f}' \
       /sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-functions
   ```

   An illegal name makes the hog fail at boot, which can drop the whole node including
   the tristate setting for every pad in it.

5. `sudo ./install.sh`, reboot, `sudo ./verify.sh <line> 15`, meter the pin.

Do not add a pad that is already a PWM you rely on (15, 33) without deciding which one
wins, and do not hand-write pin numbers anywhere else in the tree: the heartbeat's line
is a command-line argument for exactly this reason.
