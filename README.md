# VL53L9-Arduino

Experimental Arduino-Pico / RP2350 bring-up work for STMicroelectronics'
VL53L9CX time-of-flight depth sensor module.

The immediate hardware target is an ST X-NUCLEO-53L9A1 board mated to an
Adafruit Metro RP2350. The project is currently focused on proving the I3C
transport and then wrapping or porting ST's VL53L9 driver stack far enough to
initialize the sensor and read frames.

This is active bring-up code, not a finished Arduino library.

## Current Status

Implemented so far:

- PlatformIO project for RP2350 using the Earle Philhower Arduino-Pico core.
- PIO-based I3C controller integration from I3CBlaster.
- Pre-build `pioasm` generation of `i3c.pio.h`.
- Configurable I3C SDA/SCL pins, PIO instance, state machine, drive strength,
  and interrupt-locking behavior.
- Hardware bring-up sketch for the X-NUCLEO-53L9A1 + Metro RP2350 pinout.
- 12 MHz `HOST_CLKIN` PWM output generation.
- XSHUT reset sequencing.
- Initial I3C validation path: RSTDAA, ENTDAA, dynamic address ACK check, and
  simple private SDR register reads.

Not done yet:

- Hardware validation of the current bring-up sketch.
- Full ST `vl53l9_platform.c` replacement for Arduino-Pico.
- Full `vl53l9_init()` execution and firmware patch upload.
- Frame acquisition and post-processing.
- Interrupt/IBI-driven frame-ready handling.

## Hardware Target

Current default wiring is for a directly mated X-NUCLEO-53L9A1 and Adafruit
Metro RP2350:

```text
VL53 HOST XSHUT  -> Metro GPIO0
VL53 HOST INTR   -> Metro GPIO1
VL53 HOST CLKIN  -> Metro GPIO11, 12 MHz PWM output
VL53 HOST SYNCIN -> Metro GPIO44, driven high
VL53 HOST SDA    -> Metro GPIO20
VL53 HOST SCL    -> Metro GPIO21
```

The I3C PIO code currently requires adjacent SDA/SCL pins. GPIO20/GPIO21 satisfy
that requirement.

The X-NUCLEO board used for this work provides 3.3 V-safe host I/O through its
level-shifting circuitry. Re-check voltage compatibility before using a
different breakout or custom wiring.

## Repository Layout

```text
VL53L9-Pico/
  platformio.ini
    RP2350 Arduino-Pico build configuration.

  src/main.cpp
    Current hardware validation sketch.

  lib/i3c/
    RP2040/RP2350 PIO I3C transport derived from I3CBlaster, plus local
    integration notes.

  scripts/gen_i3c_pio.py
    PlatformIO pre-build script that generates i3c.pio.h.
```

The original ST package is intentionally not vendored in this repository. For
local reference work, download X-CUBE-53L9A1 from ST and place it under:

```text
STMicro_source/STM32CubeExpansion_53L9A1_V1.0.0/
```

`STMicro_source/` is ignored by git.

## Build

Install PlatformIO, then build from the `VL53L9-Pico` directory:

```sh
cd VL53L9-Pico
pio run
```

The generated UF2 is written under:

```text
VL53L9-Pico/.pio/build/rp2350/firmware.uf2
```

The project currently uses:

```ini
board = adafruit_metro_rp2350
framework = arduino
board_build.core = earlephilhower
```

## Configuration

Board wiring and conservative bring-up settings are controlled from
`VL53L9-Pico/platformio.ini`:

```ini
-DVL53L9_HOST_XSHUT_PIN=0
-DVL53L9_HOST_INTR_PIN=1
-DVL53L9_HOST_CLKIN_PIN=11
-DVL53L9_HOST_CLKIN_ENABLE=1
-DVL53L9_HOST_CLKIN_HZ=12000000
-DVL53L9_HOST_SYNCIN_PIN=44
-DVL53L9_HOST_SYNCIN_ENABLE=1
-DVL53L9_I3C_SDA_PIN=20
-DVL53L9_I3C_SCL_PIN=21
-DVL53L9_I3C_PIO_INDEX=0
-DVL53L9_I3C_SM=1
-DVL53L9_I3C_DRIVE_STRENGTH_MA=12
-DVL53L9_I3C_CLK_KHZ=1000
-DVL53L9_I3C_DISABLE_INTERRUPTS=1
-DVL53L9_I3C_DYNAMIC_ADDRESS=0x52
```

For first hardware tests, leave the I3C clock at 1 MHz and interrupt locking
enabled. Faster clocks and unlocked transfers should wait until ENTDAA and
small register reads are reliable on a logic analyzer.

## Bring-Up Test

After flashing, open the serial monitor at 115200 baud:

```sh
pio device monitor -b 115200
```

Expected stages in the serial log:

```text
VL53L9CX RP2350 I3C hardware bring-up
HOST_CLKIN PWM started ...
I3C ready ...
Resetting VL53L9CX with XSHUT
Starting I3C validation
RSTDAA: OK(...)
ENTDAA: OK(...)
Dynamic address ACK: OK(...)
VL53L9 model ID: ...
ROM revision: ...
Patch revision before boot patch: ...
System FSM state: 1 (READY_TO_BOOT)
```

If ENTDAA fails, first checks are:

- Confirm the X-NUCLEO board is configured to use the host clock source expected
  by the ST example.
- Confirm GPIO11 is producing a 12 MHz clock.
- Confirm SDA/SCL are GPIO20/GPIO21 and both idle high.
- Keep I3C at 1 MHz until the dynamic address assignment is stable.

## Design Notes

In the downloaded ST package, the portable VL53L9 core lives in:

```text
STMicro_source/STM32CubeExpansion_53L9A1_V1.0.0/Drivers/BSP/Components/vl53l9
```

The STM32-specific platform implementation lives in:

```text
STMicro_source/STM32CubeExpansion_53L9A1_V1.0.0/Utilities/vl53l9-common/vl53l9/vl53l9_platform.c
```

The RP2350 port will need to replace that platform layer with Arduino-Pico
functions backed by the PIO I3C transport. The ST platform register protocol is
straightforward: private SDR writes send a 16-bit register address followed by
payload bytes, and private SDR reads write the 16-bit register address then read
back the requested payload.

The full `vl53l9_init()` path writes board configuration, uploads the firmware
patch, boots the device, checks patch revision, and applies default settings.
The current sketch intentionally stops before that full initialization so the
transport can be validated in isolation.

## Licensing And Provenance

This repository combines code from multiple sources:

- I3CBlaster-derived PIO I3C code under the MIT license retained in
  `VL53L9-Pico/lib/i3c`.
- Local integration, build, and bring-up code for this RP2350 experiment.

ST's X-CUBE-53L9A1 package is used as a local reference for this effort but is
not checked in by default. Some ST components are listed as BSD-3-Clause in
ST's package SBOM, while other package components are covered by ST's SLA. Review
the upstream package license before vendoring, redistributing, modifying, or
copying ST-provided components into this repository.

## Roadmap

- Validate the current transport sketch on the Metro RP2350 + X-NUCLEO hardware.
- Add an Arduino-Pico `vl53l9_platform.c` implementation using `i3c_hl`.
- Pull the ST core driver into the PlatformIO build without STM32 HAL
  dependencies.
- Run `vl53l9_init()` and verify the patch revision after firmware upload.
- Add a minimal frame-read example.
- Decide whether frame-ready should use the interrupt pin, I3C IBI, or both.
