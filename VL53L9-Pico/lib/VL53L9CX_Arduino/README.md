# VL53L9CX_Arduino

Arduino-Pico helper wrapper for the local VL53L9CX bring-up work.

This library wraps the staged ST `vl53l9` driver and the local
`vl53l9_arduino` platform callbacks with a C++ class that owns the ST device
handle, ranging configuration calls, stream lifecycle, raw frame waiting and
reading, and lightweight raw-frame parsing/summary helpers.

It is not a standalone distributable Arduino library yet. It still requires:

- the local ST source staging done by `scripts/gen_st_vl53l9.py`
- the local `vl53l9_arduino` platform callback library
- an initialized I3C transport with a dynamic address already assigned

Minimal use after I3C validation:

```cpp
#include <VL53L9CX.h>

VL53L9CX sensor;

sensor.setAddress(0x52);
sensor.setPowerConfig(VDDA_2V8, VDDIO_1V8, 12000000);
sensor.setInterruptPin(1);

int status = sensor.init();
if (status != VL53L9_ERROR_NONE) {
  // inspect sensor.lastPlatformError()
}

VL53L9CXRangingConfig cfg;
cfg.sync_mode = VL53L9_SYNC_AUTONOMOUS;
cfg.power_mode = VL53L9_POWER_REGULAR;
cfg.context = VL53L9_CONTEXT_SHORT;
cfg.binning = 12;
cfg.frame_period_us = 250000;
cfg.exposure_ms = 5;

sensor.configureRanging(cfg);
sensor.attachFrameInterrupt(VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_HIGH);
sensor.start();

uint16_t raw_size = 0;
sensor.getRawBufferSize(cfg.binning, &raw_size);

uint8_t frame[VL53L9CX::rawBufferSizeForBinning(12)];
VL53L9CXFrameWaitResult wait;
status = sensor.readFrameAfterWait(frame,
                                   raw_size,
                                   2000,
                                   &wait,
                                   VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_HIGH);

sensor.stop();
```

Raw-frame helpers are also available:

```cpp
VL53L9CXRawFrame raw_frame(frame, raw_size, cfg.binning);

VL53L9CXRawFrameSummary summary;
if (raw_frame.summary(&summary)) {
  Serial.println(summary.center_depth);
  Serial.println(raw_frame.depthAt(4, 3));
  Serial.println(raw_frame.amplitudeAt(4, 3));
  Serial.println(raw_frame.ambientAt(4, 3));
}

raw_frame.printSummary(Serial, 1);
```

## Frame Handling

The ST driver exposes frame-ready as a register bit and clears/release the frame
ready indication when `vl53l9_get_frame()` acknowledges the frame. The wrapper
keeps that behavior: GPIO interrupt modes only decide when to check; the ST
frame-ready register remains the source of truth.

Supported wait modes:

- `VL53L9CX_FRAME_WAIT_POLL`: poll `vl53l9_poll_frame()` at the configured
  interval.
- `VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_LOW`: treat the configured interrupt pin as
  an active-low hint, then confirm with `vl53l9_poll_frame()`.
- `VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_HIGH`: same, but active-high.

The current X-NUCLEO-53L9A1 + Metro RP2350 smoke test defaults to GPIO
active-high with periodic register-poll fallback. GPIO modes remain hints; every
frame read is still gated by the ST frame-ready register.

When `attachFrameInterrupt()` is used with a GPIO wait mode, the wrapper attaches
a small edge ISR and latches that an interrupt happened. `waitForFrame()` reports
both the sampled GPIO level and the latched interrupt state through
`VL53L9CXFrameWaitResult`. The current implementation has one global ISR latch,
so it is intended for one VL53L9CX instance.
