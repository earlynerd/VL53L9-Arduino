# VL53L9-Arduino

Experimental Arduino-Pico / RP2350 bring-up work for STMicroelectronics'
VL53L9CX time-of-flight depth sensor module.

The immediate hardware target is an ST X-NUCLEO-53L9A1 board mated to an
Adafruit Metro RP2350. The project has now proven the I3C transport, ST driver
initialization, firmware patch upload, and raw frame reads on that hardware.

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
- Opt-in Arduino-Pico platform layer for ST's portable `vl53l9` driver.
- Opt-in build environment that copies selected ST component files from the
  local ignored ST package into the PlatformIO build directory.
- Arduino-style `VL53L9CX_Arduino` wrapper library for ST init, ranging
  configuration, stream lifecycle, frame-ready waiting, raw frame reads, and
  raw frame geometry helpers.
- Opt-in raw ranging smoke test after ST initialization: configure a conservative
  profile, start streaming, poll frame-ready, read a few I3C frames, print a
  compact depth summary, then stop streaming.
- Hardware-validated `rp2350_st_driver` path on the X-NUCLEO-53L9A1 + Metro
  RP2350: ENTDAA, dynamic address ACK, ST `vl53l9_init()`, firmware patch
  boot, device ID readback, ranging configuration, three raw frame reads, and
  `vl53l9_stop()`.
- IBI drain-and-retry handling in the Arduino platform wrapper so ST driver
  reads/writes can tolerate retryable I3C arbitration events.

Not done yet:

- Post-processing into finalized depth products.
- Interrupt/IBI-driven frame-ready handling.
- Higher-speed I3C characterization above 2 MHz.

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

  lib/vl53l9_arduino/
    Arduino-Pico platform callbacks for ST's portable VL53L9 driver.

  lib/VL53L9CX_Arduino/
    Arduino-style C++ wrapper around the local ST driver platform port.

  scripts/gen_i3c_pio.py
    PlatformIO pre-build script that generates i3c.pio.h.

  scripts/gen_st_vl53l9.py
    Optional pre-build script that stages local ST driver files for the
    rp2350_st_driver environment.
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

This builds the default `rp2350` environment, which validates the transport and
stops before calling ST's full driver.

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

To build the opt-in ST driver initialization path:

```sh
pio run -e rp2350_st_driver
```

That environment requires the local ST package path described above. It copies
only the required ST component files into `.pio/build/.../generated/vl53l9_st`
and compiles them from there.

After the same transport checks pass, `rp2350_st_driver` continues through
`vl53l9_init()` and then runs a short raw-frame ranging smoke test.

## Arduino Wrapper Library

`VL53L9-Pico/lib/VL53L9CX_Arduino` is the first Arduino-style packaging layer.
It is importable from sketches with:

```cpp
#include <VL53L9CX.h>
```

The wrapper owns the ST device handle after I3C dynamic address assignment and
provides methods for `init()`, ranging configuration, `start()`,
`waitForFrame()`, `readFrame()`, `ackFrame()`, and `stop()`. It also exposes
raw frame geometry and parsing helpers such as `rawBufferSizeForBinning()`,
`VL53L9CXRawFrame`, `summarizeRawFrame()`, and `printRawFrameSummary()`.

This is still a local project library rather than a standalone Arduino Library
Manager package. It depends on the generated/staged ST headers from
`gen_st_vl53l9.py`, so the original ST source package must remain available
locally for the `rp2350_st_driver` build.

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
-DVL53L9_I3C_DISABLE_INTERRUPTS=0
-DVL53L9_I3C_DYNAMIC_ADDRESS=0x52
-DVL53L9_RANGING_SYNC_MODE=VL53L9_SYNC_AUTONOMOUS
-DVL53L9_RANGING_POWER_MODE=VL53L9_POWER_REGULAR
-DVL53L9_RANGING_CONTEXT=VL53L9_CONTEXT_SHORT
-DVL53L9_RANGING_BINNING=12
-DVL53L9_RANGING_FRAME_PERIOD_US=250000
-DVL53L9_RANGING_EXPOSURE_MS=5
-DVL53L9_RANGING_SAMPLE_COUNT=3
-DVL53L9_RANGING_FRAME_TIMEOUT_MS=2000
-DVL53L9_RANGING_FRAME_POLL_INTERVAL_MS=5
-DVL53L9_RANGING_FRAME_WAIT_MODE=VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_HIGH
-DVL53L9_RANGING_ATTACH_HOST_INTR=1
```

The hardware-validated setting for the X-NUCLEO-53L9A1 + Metro RP2350 keeps
interrupt locking disabled. With interrupt locking enabled, ST driver init has
been observed to fail on this setup. The default I3C clock remains 1 MHz for
margin; hardware testing has been stable at speeds up to 2 MHz, while higher
rates still need transport work or signal-integrity investigation.

The first ranging test defaults to binning 12, which keeps each raw frame small
while still exercising ST's `vl53l9_start()`, `vl53l9_poll_frame()`,
`vl53l9_get_frame()`, and `vl53l9_stop()` path. Set
`VL53L9_ENABLE_RANGING_SAMPLE=0` in `rp2350_st_driver` if you want the ST-driver
build to stop immediately after `vl53l9_init()`.

### Parameter Notes

Host-board parameters:

- `VL53L9_HOST_XSHUT_PIN`: GPIO connected to the sensor reset/shutdown pin.
  The sketch drives it low, then high, before I3C discovery.
- `VL53L9_HOST_INTR_PIN`: GPIO connected to the sensor interrupt output. The
  current smoke test can poll the driver only, or use this pin as a frame-ready
  hint and confirm readiness through the ST register before reading.
- `VL53L9_HOST_CLKIN_PIN`, `VL53L9_HOST_CLKIN_ENABLE`,
  `VL53L9_HOST_CLKIN_HZ`: optional host clock output. The X-NUCLEO setup uses
  a 12 MHz PWM clock on GPIO11, matching the ST reference configuration.
- `VL53L9_HOST_SYNCIN_PIN`, `VL53L9_HOST_SYNCIN_ENABLE`: sync input pin driven
  high for the current autonomous smoke test. Manual or externally synchronized
  modes may need different handling later.

I3C transport parameters:

- `VL53L9_I3C_SDA_PIN`, `VL53L9_I3C_SCL_PIN`: PIO I3C pins. The current PIO
  program requires `SCL = SDA + 1`.
- `VL53L9_I3C_PIO_INDEX`, `VL53L9_I3C_SM`: RP2350 PIO block and state machine
  used by the I3C controller.
- `VL53L9_I3C_DRIVE_STRENGTH_MA`: GPIO drive strength. The default is 12 mA for
  sharper I3C edges through the X-NUCLEO level shifter.
- `VL53L9_I3C_CLK_KHZ`: I3C SDR clock in kHz. The default 1 MHz is intentionally
  conservative. On the current X-NUCLEO-53L9A1 + Metro RP2350 hardware, ST init
  and raw frame reads have been observed working at up to 2 MHz. Faster rates
  are not yet reliable.
- `VL53L9_I3C_DISABLE_INTERRUPTS`: wraps low-level PIO transfers in interrupt
  locking when set to `1`. The hardware-validated default is `0`. On this setup,
  `1` prevents the ST driver init/ranging path from completing reliably.
- `VL53L9_I3C_DYNAMIC_ADDRESS`: dynamic I3C address assigned during ENTDAA.
  `0x52` matches ST's default address convention.

Ranging-profile parameters:

- `VL53L9_RANGING_SYNC_MODE`: frame timing mode. `VL53L9_SYNC_AUTONOMOUS` lets
  the sensor produce frames at the configured period. `VL53L9_SYNC_MANUAL`
  requires the host to call `vl53l9_trigger_frame()` for each frame. The ST API
  also exposes `VL53L9_SYNC_SLAVE` for external synchronization.
- `VL53L9_RANGING_POWER_MODE`: sensor power/performance mode. ST exposes
  `VL53L9_POWER_REGULAR`, `VL53L9_POWER_LOW`, and `VL53L9_POWER_ULTRA_LOW`.
  Lower-power modes may reduce current draw, but should be treated as
  measurement-quality and timing tradeoffs until characterized.
- `VL53L9_RANGING_CONTEXT`: selects ST's short or long ranging context. The
  context changes the programmed VCSEL/blanking/shot sequence and calibration
  offsets used by the driver. Short context is a conservative first test; long
  context is available as `VL53L9_CONTEXT_LONG`.
- `VL53L9_RANGING_BINNING`: spatial binning factor. Larger values combine more
  sensing area into each output zone. That reduces spatial resolution and raw
  I3C bandwidth, but generally increases signal per zone and can improve
  robustness/SNR for a bring-up test.
- `VL53L9_RANGING_FRAME_PERIOD_US`: requested frame period in autonomous mode.
  ST's driver accepts 10,000 us to 1,000,000 us. It must be long enough for the
  selected exposure and readout path.
- `VL53L9_RANGING_EXPOSURE_MS`: exposure/integration time for the selected
  context. ST's driver accepts 1 ms to 30 ms. Longer exposure can increase
  return signal, but increases frame time requirements and can saturate or
  behave poorly under high ambient light.
- `VL53L9_RANGING_SAMPLE_COUNT`: number of frames read by the smoke test before
  it stops streaming.
- `VL53L9_RANGING_FRAME_TIMEOUT_MS`: host-side timeout while waiting for each
  frame-ready indication.
- `VL53L9_RANGING_FRAME_POLL_INTERVAL_MS`: delay between frame-ready register
  polls in the wrapper's wait loop.
- `VL53L9_RANGING_FRAME_WAIT_MODE`: frame wait strategy. The current
  hardware-validated default is `VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_HIGH`, which
  uses `HOST_INTR` as a hint and still confirms readiness with the ST register
  before reading the frame. `VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_LOW` is available
  if the interrupt polarity changes, and `VL53L9CX_FRAME_WAIT_POLL` ignores the
  pin and uses only ST's frame-ready register.
- `VL53L9_RANGING_ATTACH_HOST_INTR`: when set to `1`, GPIO wait modes attach a
  small `HOST_INTR` edge ISR and use its latch as a frame-ready hint. Poll mode
  ignores this flag. The ST frame-ready register is still checked before each
  frame read.

Binning deserves special attention. In the ST driver, accepted working binning
values are currently `2`, `4`, `6`, `8`, `12`, and `24`. The header mentions
`3`, but this source version's `vl53l9_set_binning()` implementation does not
accept it. The effective output shapes are:

```text
binning  raw frame   useful/cropped output  raw I3C bytes
2        54x42       54x42                  14842
4        24x24       24x20                  3844
6        18x14       18x14                  1738
8        12x10       12x10                  880
12       8x8         8x6                    516
24       4x4         4x4                    204
```

Changing binning is more than a buffer-size change. It changes the zone geometry
and crop settings, the DSS mode and effective-SPAD scaling used by
post-processing, the amount of I3C data per frame, and the practical tradeoff
between spatial detail and per-zone signal.

Other ST driver configurables not yet exposed as first-class build flags include
I3C/CSI-2 output selection, interrupt-pad versus in-band signaling, interrupt
pad electrical mode, I3C communication instance ID, and the sensor-side VDDA,
VDDIO, and external clock values used during `vl53l9_init()`. The current port
hard-codes the X-NUCLEO-oriented power/clock defaults in the Arduino platform
wrapper and uses I3C output with interrupt-pad signaling from ST's default init
path.

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

## ST Driver And Ranging Test

The opt-in `rp2350_st_driver` environment runs additional serial-log stages:

```text
Starting ST vl53l9_init()
vl53l9_init: 0 (OK)
vl53l9_get_device_id: 0 (OK), device ID ...
Ranging config: sync autonomous, power regular, context short, binning 12, ...
vl53l9_start: 0 (OK)
Frame 1 ready after ..., polls=..., gpio_active_seen=..., latched=..., irq_count=...
vl53l9_get_frame: 0 (OK)
Frame 1: sensor counter ...
Depth summary: ...
Depth grid, raw depth units:
...
vl53l9_stop: 0 (OK)
```

This path is hardware-validated on the direct X-NUCLEO-53L9A1 + Metro RP2350
setup with interrupt locking disabled and I3C at or below 2 MHz. The printed
depth grid is parsed directly from ST's raw I3C frame layout and is meant to
validate payload shape, not to replace ST's full post-processing pipeline.

## Raw Frame Format And Post-Processing

The I3C frame returned by ST's `vl53l9_get_frame()` is not a per-pixel histogram.
The sensor firmware has already reduced the optical timing data into per-pixel
raw measurement products. In the local ST sources, the I3C raw stream is named
`3DMD` and is laid out as:

```text
distance block   N * 2 bytes  15-bit raw distance plus one main/flag bit
amplitude block  N * 2 bytes  raw return signal amplitude
ambient block    N * 2 bytes  raw ambient/background signal
DSS LUT block    N / 2 bytes  4-bit DSS/aperture LUT id per pixel
status block     100 bytes    frame metadata and sensor status
```

`N` is the raw pixel count for the selected binning. For example, binning 12 has
a raw `8x8` frame, so the frame read is `64*6 + 32 + 100 = 516` bytes before
the sketch crops/prints the `8x6` useful region. Binning 4 is raw `24x24`,
cropped to `24x20`, and produces a 3844-byte I3C raw frame. Binning 2 is
`54x42` and produces a 14842-byte I3C raw frame.

The DSS field is not depth data by itself. It is a packed lookup index; ST's
post-processing maps it through calibration coefficients to estimate each
pixel's effective SPAD/aperture value. The 100-byte status block carries frame
counter, temperature, reference measurements, frame size, sync/power/context
settings, binning, crop settings, error state, frame period, and shot-count
metadata used by the processing pipeline.

ST's post-processing middleware takes the raw `3DMD` stream plus the OTP
calibration buffer returned by `vl53l9_get_calib_data()`. It can produce
processed depth, amplitude, ambient, confidence, reflectance, status, and
optionally point-cloud or 16-bit depth outputs. Its internal pipeline extracts
the raw blocks, applies OTP-derived calibration maps, does temporal noise
reduction, computes confidence, normalizes signal/ambient rates, estimates
reflectance, converts radial distance to perpendicular depth, computes Dmax,
applies sharpener and flying-pixel filtering, performs final distance validity
checks, then emits the requested output stream.

The current Arduino smoke test stops before that middleware. It reads the raw
blocks directly and prints simple summaries so we can validate boot, streaming,
frame-ready, and large I3C reads before porting or wrapping the heavier
post-processing stack. In the ST package currently used as reference, the full
transform middleware advertises I3C raw input sizes for binning 2 and binning 4;
the default binning-12 smoke test is intentionally smaller and is only for
transport validation.

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

The default `rp2350` sketch intentionally stops before `vl53l9_init()` so the
transport can be validated in isolation. The `rp2350_st_driver` environment
continues into ST's full `vl53l9_init()` path after the same transport checks.
That path writes board configuration, uploads the firmware patch, boots the
device, checks patch revision, and applies default settings.

The I3C transport is already isolated as a local PlatformIO library under
`VL53L9-Pico/lib/i3c`. It is a good candidate for a separate repository later,
but keeping it in-tree is useful while the RP2350 timing behavior, IBI handling,
and practical clock-rate limits are still changing with VL53L9 hardware tests.

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

- Add post-processing or a higher-level Arduino API for depth products.
- Decide whether frame-ready should use the interrupt pin, I3C IBI, or both.
- Characterize and improve I3C reliability above 2 MHz.
