#include <Arduino.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "i3c_hl.h"

#ifndef VL53L9_ENABLE_ST_DRIVER
#define VL53L9_ENABLE_ST_DRIVER 0
#endif

#ifndef VL53L9_ENABLE_RANGING_SAMPLE
#define VL53L9_ENABLE_RANGING_SAMPLE VL53L9_ENABLE_ST_DRIVER
#endif

#if VL53L9_ENABLE_ST_DRIVER
#include "VL53L9CX.h"
#endif

#ifndef VL53L9_HOST_XSHUT_PIN
#define VL53L9_HOST_XSHUT_PIN 0
#endif

#ifndef VL53L9_HOST_INTR_PIN
#define VL53L9_HOST_INTR_PIN 1
#endif

#ifndef VL53L9_HOST_CLKIN_PIN
#define VL53L9_HOST_CLKIN_PIN 11
#endif

#ifndef VL53L9_HOST_CLKIN_ENABLE
#define VL53L9_HOST_CLKIN_ENABLE 1
#endif

#ifndef VL53L9_HOST_CLKIN_HZ
#define VL53L9_HOST_CLKIN_HZ 12000000
#endif

#ifndef VL53L9_HOST_SYNCIN_PIN
#define VL53L9_HOST_SYNCIN_PIN 44
#endif

#ifndef VL53L9_HOST_SYNCIN_ENABLE
#define VL53L9_HOST_SYNCIN_ENABLE 1
#endif

#ifndef VL53L9_I3C_SDA_PIN
#define VL53L9_I3C_SDA_PIN 20
#endif

#ifndef VL53L9_I3C_SCL_PIN
#define VL53L9_I3C_SCL_PIN 21
#endif

#ifndef VL53L9_I3C_PIO_INDEX
#define VL53L9_I3C_PIO_INDEX 0
#endif

#ifndef VL53L9_I3C_SM
#define VL53L9_I3C_SM 1
#endif

#ifndef VL53L9_I3C_DRIVE_STRENGTH_MA
#define VL53L9_I3C_DRIVE_STRENGTH_MA 12
#endif

#ifndef VL53L9_I3C_CLK_KHZ
#define VL53L9_I3C_CLK_KHZ 2000
#endif

#ifndef VL53L9_I3C_DISABLE_INTERRUPTS
#define VL53L9_I3C_DISABLE_INTERRUPTS 0
#endif

#ifndef VL53L9_I3C_DYNAMIC_ADDRESS
#define VL53L9_I3C_DYNAMIC_ADDRESS 0x52
#endif

#ifndef VL53L9_RANGING_BINNING
#define VL53L9_RANGING_BINNING 12
#endif

#ifndef VL53L9_RANGING_FRAME_PERIOD_US
#define VL53L9_RANGING_FRAME_PERIOD_US 250000
#endif

#ifndef VL53L9_RANGING_EXPOSURE_MS
#define VL53L9_RANGING_EXPOSURE_MS 5
#endif

#ifndef VL53L9_RANGING_SAMPLE_COUNT
#define VL53L9_RANGING_SAMPLE_COUNT 3
#endif

#ifndef VL53L9_RANGING_FRAME_TIMEOUT_MS
#define VL53L9_RANGING_FRAME_TIMEOUT_MS 2000
#endif

#ifndef VL53L9_RANGING_FRAME_POLL_INTERVAL_MS
#define VL53L9_RANGING_FRAME_POLL_INTERVAL_MS 5
#endif

#ifndef VL53L9_RANGING_FRAME_WAIT_MODE
#define VL53L9_RANGING_FRAME_WAIT_MODE VL53L9CX_FRAME_WAIT_POLL
#endif

#ifndef VL53L9_RANGING_ATTACH_HOST_INTR
#define VL53L9_RANGING_ATTACH_HOST_INTR 1
#endif

#ifndef VL53L9_RANGING_SYNC_MODE
#define VL53L9_RANGING_SYNC_MODE VL53L9_SYNC_AUTONOMOUS
#endif

#ifndef VL53L9_RANGING_POWER_MODE
#define VL53L9_RANGING_POWER_MODE VL53L9_POWER_REGULAR
#endif

#ifndef VL53L9_RANGING_CONTEXT
#define VL53L9_RANGING_CONTEXT VL53L9_CONTEXT_SHORT
#endif

namespace {
constexpr uint8_t kHostXshutPin = VL53L9_HOST_XSHUT_PIN;
constexpr uint8_t kHostIntrPin = VL53L9_HOST_INTR_PIN;
constexpr uint8_t kHostClkInPin = VL53L9_HOST_CLKIN_PIN;
constexpr bool kHostClkInEnable = VL53L9_HOST_CLKIN_ENABLE != 0;
constexpr uint32_t kHostClkInHz = VL53L9_HOST_CLKIN_HZ;
constexpr uint8_t kHostSyncInPin = VL53L9_HOST_SYNCIN_PIN;
constexpr bool kHostSyncInEnable = VL53L9_HOST_SYNCIN_ENABLE != 0;

constexpr uint8_t kI3cSdaPin = VL53L9_I3C_SDA_PIN;
constexpr uint8_t kI3cSclPin = VL53L9_I3C_SCL_PIN;
constexpr uint8_t kI3cPioIndex = VL53L9_I3C_PIO_INDEX;
constexpr uint8_t kI3cStateMachine = VL53L9_I3C_SM;
constexpr uint8_t kI3cDriveStrengthMa = VL53L9_I3C_DRIVE_STRENGTH_MA;
constexpr uint32_t kI3cClockKhz = VL53L9_I3C_CLK_KHZ;
constexpr bool kI3cDisableInterrupts = VL53L9_I3C_DISABLE_INTERRUPTS != 0;
constexpr uint8_t kI3cDynamicAddress = VL53L9_I3C_DYNAMIC_ADDRESS;
constexpr bool kEnableStDriver = VL53L9_ENABLE_ST_DRIVER != 0;
constexpr bool kEnableRangingSample = VL53L9_ENABLE_RANGING_SAMPLE != 0;

constexpr uint16_t kRegModelId = 0x0000;
constexpr uint16_t kRegRomRevision = 0x000a;
constexpr uint16_t kRegPatchRevision = 0x000c;
constexpr uint16_t kRegSystemFsm = 0x008c;

static_assert(kI3cSclPin == kI3cSdaPin + 1, "I3C PIO requires SCL = SDA + 1");
static_assert(kI3cDynamicAddress <= 0x7f, "I3C dynamic address must be 7-bit");

#if VL53L9_ENABLE_ST_DRIVER
constexpr uint8_t kRangingBinning = VL53L9_RANGING_BINNING;
constexpr uint32_t kRangingFramePeriodUs = VL53L9_RANGING_FRAME_PERIOD_US;
constexpr uint16_t kRangingExposureMs = VL53L9_RANGING_EXPOSURE_MS;
constexpr uint16_t kRangingSampleCount = VL53L9_RANGING_SAMPLE_COUNT;
constexpr uint32_t kRangingFrameTimeoutMs = VL53L9_RANGING_FRAME_TIMEOUT_MS;
constexpr uint16_t kRangingFramePollIntervalMs = VL53L9_RANGING_FRAME_POLL_INTERVAL_MS;
constexpr vl53l9_sync_mode_t kRangingSyncMode = VL53L9_RANGING_SYNC_MODE;
constexpr vl53l9_power_mode_t kRangingPowerMode = VL53L9_RANGING_POWER_MODE;
constexpr vl53l9_context_t kRangingContext = VL53L9_RANGING_CONTEXT;
constexpr VL53L9CXFrameWaitMode kRangingFrameWaitMode = VL53L9_RANGING_FRAME_WAIT_MODE;
constexpr bool kRangingAttachHostIntr = VL53L9_RANGING_ATTACH_HOST_INTR != 0;

constexpr uint16_t kRangingRawBufferCapacity = VL53L9CX::rawBufferSizeForBinning(kRangingBinning);

static_assert(VL53L9CX::rawResolutionForBinning(kRangingBinning) > 0, "Unsupported VL53L9_RANGING_BINNING");
static_assert(kRangingRawBufferCapacity > VL53L9_STATUS_SIZE, "Invalid VL53L9 raw frame buffer size");

VL53L9CX gSensor;
uint8_t gFrameBuffer[kRangingRawBufferCapacity];
#endif

void printHex8(uint8_t value)
{
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void printHex16(uint16_t value)
{
  Serial.print("0x");
  printHex8(static_cast<uint8_t>(value >> 8));
  printHex8(static_cast<uint8_t>(value));
}

void printHex32(uint32_t value)
{
  Serial.print("0x");
  printHex8(static_cast<uint8_t>(value >> 24));
  printHex8(static_cast<uint8_t>(value >> 16));
  printHex8(static_cast<uint8_t>(value >> 8));
  printHex8(static_cast<uint8_t>(value));
}

void printI3cStatus(const char *label, i3c_hl_status_t status)
{
  Serial.print(label);
  Serial.print(": ");
  Serial.println(i3c_hl_get_errorstring(status));
}

#if VL53L9_ENABLE_ST_DRIVER
bool printVl53Status(const char *label, int status)
{
  Serial.print(label);
  Serial.print(": ");
  Serial.print(status);
  Serial.print(" (");
  Serial.print(VL53L9CX::errorName(status));
  Serial.println(')');
  if (status != VL53L9_ERROR_NONE) {
    const vl53l9_arduino_platform_error_t *error = gSensor.lastPlatformError();
    if ((error != nullptr) && (error->valid != 0U)) {
      VL53L9CX::printPlatformError(Serial, error);
    }
  }
  return status == VL53L9_ERROR_NONE;
}

#endif

bool startHostClock()
{
  if (!kHostClkInEnable) {
    Serial.println("HOST_CLKIN output disabled");
    return true;
  }

  if (kHostClkInHz == 0) {
    Serial.println("HOST_CLKIN frequency is invalid");
    return false;
  }

  constexpr uint16_t kPwmWrap = 1;
  const uint32_t sysHz = clock_get_hz(clk_sys);
  const float clkdiv = static_cast<float>(sysHz) / (static_cast<float>(kHostClkInHz) * (kPwmWrap + 1.0f));

  gpio_set_function(kHostClkInPin, GPIO_FUNC_PWM);
  gpio_set_slew_rate(kHostClkInPin, GPIO_SLEW_RATE_FAST);
  gpio_set_drive_strength(kHostClkInPin, GPIO_DRIVE_STRENGTH_12MA);

  const uint slice = pwm_gpio_to_slice_num(kHostClkInPin);
  const uint channel = pwm_gpio_to_channel(kHostClkInPin);
  pwm_config config = pwm_get_default_config();
  pwm_config_set_wrap(&config, kPwmWrap);
  pwm_config_set_clkdiv(&config, clkdiv);
  pwm_init(slice, &config, false);
  pwm_set_chan_level(slice, channel, 1);
  pwm_set_enabled(slice, true);

  Serial.print("HOST_CLKIN PWM started on GPIO");
  Serial.print(kHostClkInPin);
  Serial.print(" at ");
  Serial.print(kHostClkInHz);
  Serial.print(" Hz, sysclk ");
  Serial.print(sysHz);
  Serial.print(" Hz, clkdiv ");
  Serial.println(clkdiv, 4);
  return true;
}

void configureHostPins()
{
  pinMode(kHostIntrPin, INPUT);

  if (kHostSyncInEnable) {
    pinMode(kHostSyncInPin, OUTPUT);
    digitalWrite(kHostSyncInPin, HIGH);
  }

  pinMode(kHostXshutPin, OUTPUT);
  digitalWrite(kHostXshutPin, LOW);

  Serial.print("Pins: XSHUT GPIO");
  Serial.print(kHostXshutPin);
  Serial.print(", INTR GPIO");
  Serial.print(kHostIntrPin);
  Serial.print(", CLKIN GPIO");
  Serial.print(kHostClkInPin);
  Serial.print(", SYNCIN ");
  if (kHostSyncInEnable) {
    Serial.print("GPIO");
    Serial.println(kHostSyncInPin);
  } else {
    Serial.println("disabled");
  }
}

bool initI3cTransport()
{
  i3c_hl_config_t config = i3c_hl_default_config(kI3cSdaPin);
  config.scl_pin = kI3cSclPin;
  config.pio_index = kI3cPioIndex;
  config.sm = kI3cStateMachine;
  config.drive_strength_mA = kI3cDriveStrengthMa;
  config.disable_interrupts = kI3cDisableInterrupts;

  i3c_hl_status_t status = i3c_init_config(&config);
  if (status != i3c_hl_status_ok) {
    printI3cStatus("I3C init failed", status);
    return false;
  }

  status = i3c_hl_set_clkrate(kI3cClockKhz);
  if (status != i3c_hl_status_ok) {
    printI3cStatus("I3C clock setup failed", status);
    return false;
  }

  Serial.print("I3C ready: SDA GPIO");
  Serial.print(kI3cSdaPin);
  Serial.print(", SCL GPIO");
  Serial.print(kI3cSclPin);
  Serial.print(", PIO");
  Serial.print(kI3cPioIndex);
  Serial.print("/SM");
  Serial.print(kI3cStateMachine);
  Serial.print(", ");
  Serial.print(kI3cClockKhz);
  Serial.print(" kHz, interrupt locking ");
  Serial.println(kI3cDisableInterrupts ? "enabled" : "disabled");
  return true;
}

void resetSensor()
{
  Serial.println("Resetting VL53L9CX with XSHUT");
  digitalWrite(kHostXshutPin, LOW);
  delay(50);
  digitalWrite(kHostXshutPin, HIGH);
  delay(50);
  Serial.print("HOST_INTR after reset: ");
  Serial.println(digitalRead(kHostIntrPin));
}

bool pollIbiIfPending()
{
  uint8_t ibi[8] = {};
  uint32_t len = sizeof(ibi);
  i3c_hl_status_t status = i3c_hl_poll(ibi, &len);

  if (status == i3c_hl_status_no_ibi) {
    return true;
  }
  if (status != i3c_hl_status_ok) {
    printI3cStatus("I3C IBI poll failed", status);
    return false;
  }

  Serial.print("I3C IBI/HJ received:");
  for (uint32_t i = 0; i < len; ++i) {
    Serial.print(' ');
    Serial.print("0x");
    printHex8(ibi[i]);
  }
  Serial.println();
  return true;
}

bool runI3cWithIbiRetry(const char *label, i3c_hl_status_t (*operation)())
{
  i3c_hl_status_t status = operation();
  if (status == i3c_hl_status_ibi) {
    Serial.print(label);
    Serial.println(" saw pending IBI/HJ; polling then retrying");
    if (!pollIbiIfPending()) {
      return false;
    }
    status = operation();
  }

  printI3cStatus(label, status);
  return status == i3c_hl_status_ok;
}

i3c_hl_status_t rstdaaOperation()
{
  return i3c_hl_rstdaa();
}

bool assignDynamicAddress(uint8_t *pid)
{
  i3c_hl_status_t status = i3c_hl_entdaa(kI3cDynamicAddress, pid);
  if (status == i3c_hl_status_ibi) {
    Serial.println("ENTDAA saw pending IBI/HJ; polling then retrying");
    if (!pollIbiIfPending()) {
      return false;
    }
    status = i3c_hl_entdaa(kI3cDynamicAddress, pid);
  }

  printI3cStatus("ENTDAA", status);
  if (status != i3c_hl_status_ok) {
    return false;
  }

  Serial.print("Assigned dynamic address ");
  printHex8(kI3cDynamicAddress);
  Serial.print(", ENTDAA payload:");
  for (uint8_t i = 0; i < 8; ++i) {
    Serial.print(' ');
    Serial.print("0x");
    printHex8(pid[i]);
  }
  Serial.println();
  return true;
}

bool checkDynamicAddress()
{
  i3c_hl_status_t status = i3c_hl_checkack(kI3cDynamicAddress);
  printI3cStatus("Dynamic address ACK", status);
  return status == i3c_hl_status_ok;
}

bool readVl53Register(uint16_t reg, uint8_t *data, uint32_t size)
{
  uint8_t command[2] = {
      static_cast<uint8_t>(reg >> 8),
      static_cast<uint8_t>(reg),
  };
  uint32_t readSize = size;
  i3c_hl_status_t status = i3c_hl_sdr_privwriteread(
      kI3cDynamicAddress,
      command,
      sizeof(command),
      data,
      &readSize);

  if (status != i3c_hl_status_ok) {
    Serial.print("Read ");
    printHex16(reg);
    Serial.print(" failed: ");
    Serial.println(i3c_hl_get_errorstring(status));
    return false;
  }
  if (readSize != size) {
    Serial.print("Read ");
    printHex16(reg);
    Serial.print(" returned ");
    Serial.print(readSize);
    Serial.print(" bytes, expected ");
    Serial.println(size);
    return false;
  }
  return true;
}

bool readAndPrintRevisionRegisters()
{
  uint8_t modelBytes[4] = {};
  uint8_t romBytes[2] = {};
  uint8_t patchBytes[2] = {};
  uint8_t fsm = 0xff;

  if (!readVl53Register(kRegModelId, modelBytes, sizeof(modelBytes))) {
    return false;
  }
  if (!readVl53Register(kRegRomRevision, romBytes, sizeof(romBytes))) {
    return false;
  }
  if (!readVl53Register(kRegPatchRevision, patchBytes, sizeof(patchBytes))) {
    return false;
  }
  if (!readVl53Register(kRegSystemFsm, &fsm, 1)) {
    return false;
  }

  const uint32_t modelId =
      static_cast<uint32_t>(modelBytes[0]) |
      (static_cast<uint32_t>(modelBytes[1]) << 8) |
      (static_cast<uint32_t>(modelBytes[2]) << 16) |
      (static_cast<uint32_t>(modelBytes[3]) << 24);
  const uint16_t romRevision = static_cast<uint16_t>(romBytes[0]) | (static_cast<uint16_t>(romBytes[1]) << 8);
  const uint16_t patchRevision = static_cast<uint16_t>(patchBytes[0]) | (static_cast<uint16_t>(patchBytes[1]) << 8);

  Serial.print("VL53L9 model ID: ");
  printHex32(modelId);
  Serial.print(" raw");
  for (uint8_t value : modelBytes) {
    Serial.print(" 0x");
    printHex8(value);
  }
  Serial.println();

  Serial.print("ROM revision: ");
  printHex16(romRevision);
  Serial.print(" raw 0x");
  printHex8(romBytes[0]);
  Serial.print(" 0x");
  printHex8(romBytes[1]);
  Serial.println();

  Serial.print("Patch revision before boot patch: ");
  printHex16(patchRevision);
  Serial.print(" raw 0x");
  printHex8(patchBytes[0]);
  Serial.print(" 0x");
  printHex8(patchBytes[1]);
  Serial.println();

  Serial.print("System FSM state: ");
  Serial.print(fsm);
  Serial.println(fsm == 1 ? " (READY_TO_BOOT)" : "");
  return true;
}

#if VL53L9_ENABLE_ST_DRIVER
bool configureRanging(VL53L9CX *sensor, uint16_t *rawBufferSize)
{
  int status = sensor->getRawBufferSize(kRangingBinning, rawBufferSize);
  if (!printVl53Status("vl53l9_get_raw_buffer_size", status)) {
    return false;
  }
  if (*rawBufferSize > sizeof(gFrameBuffer)) {
    Serial.print("Raw frame buffer too small: ");
    Serial.print(sizeof(gFrameBuffer));
    Serial.print(" < ");
    Serial.println(*rawBufferSize);
    return false;
  }

  Serial.print("Ranging config: sync ");
  Serial.print(VL53L9CX::syncModeName(kRangingSyncMode));
  Serial.print(", power ");
  Serial.print(VL53L9CX::powerModeName(kRangingPowerMode));
  Serial.print(", context ");
  Serial.print(VL53L9CX::contextName(kRangingContext));
  Serial.print(", binning ");
  Serial.print(kRangingBinning);
  Serial.print(", output ");
  Serial.print(VL53L9CX::outputWidthForBinning(kRangingBinning));
  Serial.print('x');
  Serial.print(VL53L9CX::outputHeightForBinning(kRangingBinning));
  Serial.print(", raw ");
  Serial.print(VL53L9CX::rawWidthForBinning(kRangingBinning));
  Serial.print('x');
  Serial.print(VL53L9CX::rawHeightForBinning(kRangingBinning));
  Serial.print(", frame period ");
  Serial.print(kRangingFramePeriodUs);
  Serial.print(" us, exposure ");
  Serial.print(kRangingExposureMs);
  Serial.print(" ms, buffer ");
  Serial.print(*rawBufferSize);
  Serial.print(" bytes, wait ");
  Serial.print(VL53L9CX::frameWaitModeName(kRangingFrameWaitMode));
  Serial.print("/");
  Serial.print(kRangingFramePollIntervalMs);
  Serial.print(" ms");
  if (kRangingFrameWaitMode != VL53L9CX_FRAME_WAIT_POLL) {
    Serial.print(", irq latch ");
    Serial.print(kRangingAttachHostIntr ? "enabled" : "disabled");
  }
  Serial.println();

  status = sensor->setSyncMode(kRangingSyncMode);
  if (!printVl53Status("vl53l9_set_sync_mode", status)) {
    return false;
  }
  status = sensor->setPowerMode(kRangingPowerMode);
  if (!printVl53Status("vl53l9_set_power_mode", status)) {
    return false;
  }
  status = sensor->setFramePeriod(kRangingFramePeriodUs);
  if (!printVl53Status("vl53l9_set_frame_period", status)) {
    return false;
  }
  status = sensor->setContext(kRangingContext);
  if (!printVl53Status("vl53l9_set_context", status)) {
    return false;
  }
  status = sensor->setBinning(kRangingContext, kRangingBinning);
  if (!printVl53Status("vl53l9_set_binning", status)) {
    return false;
  }
  status = sensor->setExposure(kRangingContext, kRangingExposureMs);
  if (!printVl53Status("vl53l9_set_exposure", status)) {
    return false;
  }

  return true;
}

bool waitForRangingFrame(VL53L9CX *sensor, uint16_t frameIndex)
{
  VL53L9CXFrameWaitResult waitResult;
  const int status = sensor->waitForFrame(kRangingFrameTimeoutMs,
                                          &waitResult,
                                          kRangingFrameWaitMode,
                                          kRangingFramePollIntervalMs);
  if (status == VL53L9_ERROR_NONE) {
    Serial.print("Frame ");
    Serial.print(frameIndex);
    Serial.print(" ready after ");
    Serial.print(waitResult.elapsed_ms);
    Serial.print(" ms, HOST_INTR=");
    Serial.print(digitalRead(kHostIntrPin));
    Serial.print(", polls=");
    Serial.print(waitResult.polls);
    if (kRangingFrameWaitMode != VL53L9CX_FRAME_WAIT_POLL) {
      Serial.print(", gpio_active_seen=");
      Serial.print(waitResult.interrupt_observed_active ? "yes" : "no");
      Serial.print(", latched=");
      Serial.print(waitResult.interrupt_latched ? "yes" : "no");
      Serial.print(", irq_count=");
      Serial.print(waitResult.interrupt_count);
      Serial.print(", last_gpio=");
      Serial.print(waitResult.interrupt_level);
    }
    Serial.println();
    return true;
  }

  if (status != VL53L9_ERROR_TIMEOUT) {
    printVl53Status("vl53l9_poll_frame", status);
    return false;
  }

  Serial.print("Timed out waiting for frame ");
  Serial.print(frameIndex);
  Serial.print(" after ");
  Serial.print(waitResult.elapsed_ms);
  Serial.print(" ms, HOST_INTR=");
  Serial.print(digitalRead(kHostIntrPin));
  Serial.print(", polls=");
  Serial.println(waitResult.polls);
  return false;
}

bool runStRangingSample(VL53L9CX *sensor)
{
  if (!kEnableRangingSample) {
    Serial.println("Ranging sample disabled.");
    return true;
  }

  uint16_t rawBufferSize = 0U;
  if (!configureRanging(sensor, &rawBufferSize)) {
    return false;
  }

  if (kRangingSampleCount == 0U) {
    Serial.println("Ranging sample count is zero; skipping stream start.");
    return true;
  }

  if (kRangingFrameWaitMode != VL53L9CX_FRAME_WAIT_POLL) {
    Serial.print("HOST_INTR interrupt latch ");
    if (!kRangingAttachHostIntr) {
      Serial.println("disabled by build flag");
    } else if (sensor->attachFrameInterrupt(kRangingFrameWaitMode)) {
      Serial.println("attached");
    } else {
      Serial.println("not attached; falling back to GPIO sampling and polling");
    }
  }

  int status = sensor->start();
  if (!printVl53Status("vl53l9_start", status)) {
    return false;
  }

  bool ok = true;
  for (uint16_t frameIndex = 1; frameIndex <= kRangingSampleCount; ++frameIndex) {
    if (kRangingSyncMode == VL53L9_SYNC_MANUAL) {
      status = sensor->triggerFrame();
      if (!printVl53Status("vl53l9_trigger_frame", status)) {
        ok = false;
        break;
      }
    }

    if (!waitForRangingFrame(sensor, frameIndex)) {
      ok = false;
      break;
    }

    status = sensor->readFrame(gFrameBuffer, rawBufferSize);
    if (!printVl53Status("vl53l9_get_frame", status)) {
      ok = false;
      break;
    }

    VL53L9CXRawFrame frame(gFrameBuffer, rawBufferSize, kRangingBinning);
    frame.printSummary(Serial, frameIndex);
  }

  status = sensor->stop();
  if (!printVl53Status("vl53l9_stop", status)) {
    ok = false;
  }

  return ok;
}

bool runStDriverInit()
{
  gSensor.setAddress(kI3cDynamicAddress);
  gSensor.setPowerConfig(VDDA_2V8, VDDIO_1V8, kHostClkInHz);
  gSensor.setInterruptPin(kHostIntrPin);

  Serial.println("Starting ST vl53l9_init()");
  const int initStatus = gSensor.init();
  if (!printVl53Status("vl53l9_init", initStatus)) {
    return false;
  }

  uint32_t deviceId = 0;
  const int idStatus = gSensor.getDeviceId(&deviceId);
  Serial.print("vl53l9_get_device_id: ");
  Serial.print(idStatus);
  Serial.print(" (");
  Serial.print(VL53L9CX::errorName(idStatus));
  Serial.print(')');
  Serial.print(", device ID ");
  printHex32(deviceId);
  Serial.println();

  return idStatus == VL53L9_ERROR_NONE;
}
#endif

void runHardwareBringup()
{
  configureHostPins();
  if (!startHostClock()) {
    return;
  }
  if (!initI3cTransport()) {
    return;
  }

  resetSensor();

  Serial.println("Starting I3C validation");
  pollIbiIfPending();
  runI3cWithIbiRetry("RSTDAA", rstdaaOperation);

  uint8_t pid[8] = {};
  if (!assignDynamicAddress(pid)) {
    Serial.println("Stopping before register reads; ENTDAA did not complete");
    return;
  }
  if (!checkDynamicAddress()) {
    Serial.println("Stopping before register reads; dynamic address did not ACK");
    return;
  }
  if (!readAndPrintRevisionRegisters()) {
    Serial.println("Transport validation failed during register read");
    return;
  }

  Serial.print("Transport validation passed. ST driver init ");
  Serial.println(kEnableStDriver ? "enabled." : "disabled.");

#if VL53L9_ENABLE_ST_DRIVER
  if (!runStDriverInit()) {
    Serial.println("ST driver initialization failed");
    return;
  }

  Serial.println("ST driver initialization passed.");
  if (!runStRangingSample(&gSensor)) {
    Serial.println("ST ranging sample failed");
    return;
  }

  Serial.println("ST ranging sample passed.");
#else
  Serial.println("Build env rp2350_st_driver enables the optional vl53l9_init() path.");
#endif
}
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("VL53L9CX RP2350 I3C hardware bring-up");
  runHardwareBringup();
}

void loop()
{
  static uint32_t lastPrintMs = 0;
  const uint32_t now = millis();
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    Serial.print("HOST_INTR=");
    Serial.println(digitalRead(kHostIntrPin));
  }
}
