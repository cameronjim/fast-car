// Pico SDK glue: the ONE owner of this core's shared GPIO edge-interrupt callback.
// UNVERIFIED ON HARDWARE -- see firmware/safety_mux/README.md. Deliberately NOT unit-tested
// (it is real GPIO/interrupt hardware access, no host equivalent);
// firmware/safety_mux/logic/ is the tested half.
//
// WHY THIS EXISTS -- read before touching any GPIO interrupt code in this directory:
//
//   In the Pico SDK (2.1.0, hardware_gpio/gpio.c), gpio_set_irq_callback() and
//   gpio_set_irq_enabled_with_callback() store exactly ONE callback PER CORE
//   (`callbacks[core] = callback`). The second module to call either function silently
//   REPLACES the first module's callback for the entire GPIO bank. There is no error, no
//   warning, and nothing a host test can see.
//
//   That bug shipped in this directory: pwm_capture.c installed its handler, then
//   heartbeat_input.c installed its own, and because main() inits capture first and
//   heartbeat second, the heartbeat handler won. All three PWM capture channels would have
//   read -1.0 forever on hardware and the mux would have cut permanently -- fail-safe, but
//   completely inert. See docs/notes/safety-mux-first-build.md.
//
//   So: NO module in firmware/safety_mux/pico/ may call gpio_set_irq_callback() or
//   gpio_set_irq_enabled_with_callback() itself. Every module that wants GPIO edge
//   interrupts registers here, per GPIO, and this file owns the single callback and routes
//   each event to the right module by GPIO number.
#ifndef SAFETY_MUX_PICO_GPIO_IRQ_DISPATCH_H_
#define SAFETY_MUX_PICO_GPIO_IRQ_DISPATCH_H_

#include <stdbool.h>
#include <stdint.h>

#include "pico/types.h"  // Pico SDK's `uint` typedef, used in the signatures below

// A per-GPIO edge-interrupt handler. Called from interrupt context: it must be short and
// allocation-free, exactly like a raw SDK callback. `gpio` is always the GPIO the handler
// was registered for, and `events` is the SDK's GPIO_IRQ_* event mask for that edge.
typedef void (*GpioIrqDispatchHandler)(uint gpio, uint32_t events);

// Routes `event_mask` edges on `gpio` to `handler`, and enables those edges on that GPIO.
// Installs the single shared SDK callback on first use. Registering the same GPIO twice
// replaces its handler (last caller wins) rather than silently stealing every OTHER GPIO's
// handler, which is the failure mode this module exists to make impossible.
//
// Returns false (and registers nothing) if the handler table is full or `handler` is NULL;
// the caller decides what that means. The table is sized for every GPIO this firmware uses
// (see pico/main.c's pin defines), so a false return means someone added inputs without
// raising GPIO_IRQ_DISPATCH_MAX_HANDLERS.
bool gpio_irq_dispatch_register(uint gpio, uint32_t event_mask, GpioIrqDispatchHandler handler);

#endif  // SAFETY_MUX_PICO_GPIO_IRQ_DISPATCH_H_
