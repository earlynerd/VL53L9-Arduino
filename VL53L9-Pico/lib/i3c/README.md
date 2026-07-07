# I3C PIO Transport

This directory contains an RP2040/RP2350 PIO-based I3C controller derived from
I3CBlaster by Kai Gossner. It is being used here as the candidate low-level
transport for the VL53L9CX Arduino-Pico port.

## Generated PIO Header

`i3c_hl.c` includes `i3c.pio.h`. PlatformIO now generates that header before
the build with `scripts/gen_i3c_pio.py` and adds the generated directory to the
include path:

```text
.pio/build/<env>/generated/i3c/i3c.pio.h
```

Run the normal build command from `VL53L9-Pico`:

```sh
pio run
```

## Pin and PIO Configuration

The original `i3c_init(sda_pin)` API still works and uses the defaults in
`i3c_hl.h`. For board-specific setup, use `i3c_init_config()`:

```c
#include "i3c_hl.h"

void setup_i3c(void)
{
    i3c_hl_config_t cfg = i3c_hl_default_config(16); // SDA = GPIO16
    cfg.scl_pin = 17;                                // must be SDA + 1
    cfg.pio_index = 0;                               // 0 = pio0, 1 = pio1
    cfg.sm = 1;                                      // state machine 0..3
    cfg.drive_strength_mA = 12;                      // 2, 4, 8, or 12
    cfg.disable_interrupts = true;                   // upstream-compatible default

    i3c_init_config(&cfg);
    i3c_hl_set_clkrate(1000);                        // start conservatively
}
```

The current PIO program requires adjacent pins. `scl_pin` must be exactly
`sda_pin + 1`; non-adjacent pairs need PIO program changes.

The defaults can also be overridden from PlatformIO `build_flags` with
`I3C_HL_DEFAULT_PIO_INDEX`, `I3C_HL_DEFAULT_SM`,
`I3C_HL_DEFAULT_DRIVE_STRENGTH_MA`, and
`I3C_HL_DEFAULT_DISABLE_INTERRUPTS`.

The PIO program is copied into instruction memory at offset 0 of the selected
PIO instance. Do not share that PIO instance with another program unless the
instruction memory ownership is planned deliberately.

The same physical pins can be switched between PIO I3C and the hardware I2C
function with `i3c_hl_i2c_pinmode()`. The function now follows the configured
PIO instance when switching back to I3C.

## Interrupt Locking

Upstream I3CBlaster disables interrupts around most bus transactions to protect
PIO timing. This port keeps that behavior by default, but makes it configurable:

```c
cfg.disable_interrupts = false;
// or later:
i3c_hl_set_interrupt_locking(false);
```

Disabling the lock should be treated as experimental. Start at a low I3C clock,
verify with a logic analyzer, and watch for corrupted reads or unexpected NACKs.
The safest first VL53L9CX bring-up path is to keep locking enabled for ENTDAA,
CCC setup, and small register probes. After the sensor is responding reliably,
try disabling it at low clock speed before attempting full frame reads.

One bug from the upstream code was also fixed here: `i3c_hl_ddr_read()` no
longer returns from its zero-length parameter check after disabling interrupts.

## VL53L9CX Notes

The VL53L9CX STM32 example uses I3C private write/read operations after dynamic
address assignment. For the RP2350 port, this library should sit under the ST
platform layer replacement and provide the equivalent register write/read
transport.

Use the I3C dynamic address assigned during ENTDAA for private transfers. The
legacy VL53-style `0x29` address is for I2C mode, not I3C private SDR traffic.
