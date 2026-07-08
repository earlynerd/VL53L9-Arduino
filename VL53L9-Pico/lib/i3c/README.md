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
    cfg.disable_interrupts = false;                  // VL53L9 validated default

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
PIO timing. This port makes that behavior configurable:

```c
cfg.disable_interrupts = false;
// or later:
i3c_hl_set_interrupt_locking(false);
```

For the current X-NUCLEO-53L9A1 + Metro RP2350 setup, the hardware-validated
VL53L9 path requires interrupt locking disabled. With locking enabled,
`vl53l9_init()` has been observed to fail even though ENTDAA and small private
register reads can pass.

Keep the I3C clock conservative while validating changes. The current hardware
path has worked at 1 MHz and up to 2 MHz with interrupt locking disabled; higher
rates are not yet reliable.

One bug from the upstream code was also fixed here: `i3c_hl_ddr_read()` no
longer returns from its zero-length parameter check after disabling interrupts.

## VL53L9CX Notes

The VL53L9CX STM32 example uses I3C private write/read operations after dynamic
address assignment. For the RP2350 port, this library should sit under the ST
platform layer replacement and provide the equivalent register write/read
transport.

Use the I3C dynamic address assigned during ENTDAA for private transfers. The
legacy VL53-style `0x29` address is for I2C mode, not I3C private SDR traffic.

## X-NUCLEO-53L9A1 + Metro RP2350 Bring-Up

The current PlatformIO defaults target the direct X-NUCLEO-53L9A1 to Adafruit
Metro RP2350 connection:

```text
HOST XSHUT  -> GPIO0
HOST INTR   -> GPIO1
HOST CLKIN  -> GPIO11, 12 MHz PWM output
HOST SYNCIN -> GPIO44, driven high
HOST SDA    -> GPIO20
HOST SCL    -> GPIO21
```

The validation sketch starts the 12 MHz host clock, holds/releases XSHUT,
initializes the PIO I3C transport at the DAA clock, runs RSTDAA and ENTDAA with
dynamic address `0x52`, optionally switches to the runtime SDR clock, checks
that the address ACKs, then attempts private SDR reads of the model, ROM
revision, patch revision, and FSM registers. The default DAA and runtime clocks
are both 2 MHz; set `VL53L9_I3C_CLK_KHZ` higher while keeping
`VL53L9_I3C_DAA_CLK_KHZ=2000` to test whether high-speed private transfers work
after conservative dynamic address assignment.

The optional `rp2350_st_driver` environment continues into ST's init path. On
hardware it has completed `vl53l9_init()`, firmware patch boot, ranging
configuration, raw frame reads, and `vl53l9_stop()` when interrupt locking is
disabled and the I3C clock is between 2.0 MHz and 3.2 MHz.
