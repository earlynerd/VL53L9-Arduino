# VL53L9 Arduino-Pico Platform Layer

This directory contains the RP2350 Arduino-Pico platform glue for ST's portable
VL53L9 driver.

It intentionally does not contain ST's driver sources. The `rp2350_st_driver`
PlatformIO environment copies the required files from the local, git-ignored
`STMicro_source/` tree into the PlatformIO build directory.

## What This Layer Provides

- A small `vl53l9_arduino_device_t` descriptor with the I3C dynamic address and
  board power/clock configuration used by ST's init sequence.
- Implementations of ST's required `vl53l9_platform.h` callbacks.
- Register reads and writes backed by the PIO I3C transport in `lib/i3c`.
- Chunked private SDR writes for the firmware patch upload path.
- Chunked private SDR reads for raw frame acquisition.
- Synchronous fallback behavior for `vl53l9_read_async()`.

## Current Assumptions

- The sensor has already been reset and assigned a dynamic I3C address.
- The I3C transport has already passed simple private SDR register reads.
- The X-NUCLEO-53L9A1 sensor-side configuration follows ST's reference defaults:
  `VDDA_2V8`, `VDDIO_1V8`, and a 12 MHz external clock.
- The host-side 3.3 V-safe level shifter on the X-NUCLEO board does not imply a
  3.3 V sensor-side VDDIO setting.

## Build

Use the default environment for transport-only hardware validation:

```sh
pio run -e rp2350
```

Use the opt-in ST driver environment after the transport test works:

```sh
pio run -e rp2350_st_driver
```

The sketch uses this environment to run `vl53l9_init()` and then a short
raw-frame ranging smoke test through ST's public driver APIs.

That environment requires:

```text
STMicro_source/STM32CubeExpansion_53L9A1_V1.0.0/Drivers/BSP/Components/vl53l9
```

The copied ST files are build artifacts only and should not be committed unless
the repository licensing strategy changes.
