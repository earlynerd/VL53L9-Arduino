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
#include "vl53l9_arduino_port.h"
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
#define VL53L9_I3C_CLK_KHZ 1000
#endif

#ifndef VL53L9_I3C_DISABLE_INTERRUPTS
#define VL53L9_I3C_DISABLE_INTERRUPTS 1
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
constexpr vl53l9_sync_mode_t kRangingSyncMode = VL53L9_RANGING_SYNC_MODE;
constexpr vl53l9_power_mode_t kRangingPowerMode = VL53L9_RANGING_POWER_MODE;
constexpr vl53l9_context_t kRangingContext = VL53L9_RANGING_CONTEXT;

constexpr uint16_t rawResolutionForBinning(uint8_t binning)
{
  switch (binning) {
    case 2:
      return 54U * 42U;
    case 4:
      return 24U * 24U;
    case 6:
      return 18U * 14U;
    case 8:
      return 12U * 10U;
    case 12:
      return 8U * 8U;
    case 24:
      return 4U * 4U;
    default:
      return 0;
  }
}

constexpr uint8_t rawWidthForBinning(uint8_t binning)
{
  switch (binning) {
    case 2:
      return 54;
    case 4:
      return 24;
    case 6:
      return 18;
    case 8:
      return 12;
    case 12:
      return 8;
    case 24:
      return 4;
    default:
      return 0;
  }
}

constexpr uint8_t outputWidthForBinning(uint8_t binning)
{
  return rawWidthForBinning(binning);
}

constexpr uint8_t outputHeightForBinning(uint8_t binning)
{
  switch (binning) {
    case 2:
      return 42;
    case 4:
      return 20;
    case 6:
      return 14;
    case 8:
      return 10;
    case 12:
      return 6;
    case 24:
      return 4;
    default:
      return 0;
  }
}

constexpr uint8_t outputYOffsetForBinning(uint8_t binning)
{
  switch (binning) {
    case 4:
      return 2;
    case 12:
      return 1;
    default:
      return 0;
  }
}

constexpr uint16_t rawBufferSizeForBinning(uint8_t binning)
{
  const uint16_t resolution = rawResolutionForBinning(binning);
  return resolution == 0 ? 0 : static_cast<uint16_t>((resolution * 6U) + (resolution / 2U) + VL53L9_STATUS_SIZE);
}

constexpr uint16_t kRangingRawResolution = rawResolutionForBinning(kRangingBinning);
constexpr uint8_t kRangingRawWidth = rawWidthForBinning(kRangingBinning);
constexpr uint8_t kRangingOutputWidth = outputWidthForBinning(kRangingBinning);
constexpr uint8_t kRangingOutputHeight = outputHeightForBinning(kRangingBinning);
constexpr uint8_t kRangingOutputYOffset = outputYOffsetForBinning(kRangingBinning);
constexpr uint16_t kRangingRawBufferCapacity = rawBufferSizeForBinning(kRangingBinning);

static_assert(kRangingRawResolution > 0, "Unsupported VL53L9_RANGING_BINNING");
static_assert(kRangingRawBufferCapacity > VL53L9_STATUS_SIZE, "Invalid VL53L9 raw frame buffer size");

vl53l9_arduino_device_t gVl53Device;
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
const char *vl53l9ErrorName(int status)
{
  switch (status) {
    case VL53L9_ERROR_NONE:
      return "OK";
    case VL53L9_ERROR_PLATFORM:
      return "PLATFORM";
    case VL53L9_ERROR_INVALID_PARAM:
      return "INVALID_PARAM";
    case VL53L9_ERROR_INVALID_STATE:
      return "INVALID_STATE";
    case VL53L9_ERROR_INVALID_OPERATION:
      return "INVALID_OPERATION";
    case VL53L9_ERROR_TIMEOUT:
      return "TIMEOUT";
    case VL53L9_ERROR_INTERNAL:
      return "INTERNAL";
    default:
      return "UNKNOWN";
  }
}

bool printVl53Status(const char *label, int status)
{
  Serial.print(label);
  Serial.print(": ");
  Serial.print(status);
  Serial.print(" (");
  Serial.print(vl53l9ErrorName(status));
  Serial.println(')');
  return status == VL53L9_ERROR_NONE;
}

void printVl53PlatformError()
{
  const vl53l9_arduino_platform_error_t *error = vl53l9_arduino_get_last_error();
  if ((error == nullptr) || (error->valid == 0U)) {
    Serial.println("VL53L9 platform error detail: no I3C failure recorded");
    return;
  }

  Serial.print("VL53L9 platform error detail: op=");
  Serial.print(error->operation != nullptr ? error->operation : "unknown");
  Serial.print(", addr=");
  printHex8(error->i3c_address);
  Serial.print(", reg=");
  printHex16(error->register_address);
  Serial.print(", requested=");
  Serial.print(error->requested_size);
  Serial.print(", actual=");
  Serial.print(error->actual_size);
  Serial.print(", chunk_offset=");
  Serial.print(error->chunk_offset);
  Serial.print(", ibi_retries=");
  Serial.print(error->ibi_retries);
  Serial.print(", i3c_status=");
  Serial.print(error->i3c_status);
  Serial.print(" (");
  Serial.print(vl53l9_arduino_i3c_status_name(error->i3c_status));
  Serial.println(')');
}

const char *syncModeName(vl53l9_sync_mode_t mode)
{
  switch (mode) {
    case VL53L9_SYNC_SLAVE:
      return "slave";
    case VL53L9_SYNC_MANUAL:
      return "manual";
    case VL53L9_SYNC_AUTONOMOUS:
      return "autonomous";
    default:
      return "unknown";
  }
}

const char *powerModeName(vl53l9_power_mode_t mode)
{
  switch (mode) {
    case VL53L9_POWER_REGULAR:
      return "regular";
    case VL53L9_POWER_LOW:
      return "low";
    case VL53L9_POWER_ULTRA_LOW:
      return "ultra-low";
    default:
      return "unknown";
  }
}

const char *contextName(vl53l9_context_t context)
{
  switch (context) {
    case VL53L9_CONTEXT_SHORT:
      return "short";
    case VL53L9_CONTEXT_LONG:
      return "long";
    default:
      return "unknown";
  }
}

uint16_t readLe16(const uint8_t *data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t readLe32(const uint8_t *data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

void printPaddedDepth(uint16_t value)
{
  if (value < 10000) {
    Serial.print(' ');
  }
  if (value < 1000) {
    Serial.print(' ');
  }
  if (value < 100) {
    Serial.print(' ');
  }
  if (value < 10) {
    Serial.print(' ');
  }
  Serial.print(value);
}

uint16_t depthAtOutputPixel(const uint8_t *buffer, uint8_t x, uint8_t y)
{
  const uint16_t rawIndex =
      static_cast<uint16_t>(y + kRangingOutputYOffset) * kRangingRawWidth + x;
  return readLe16(&buffer[rawIndex * 2U]) & 0x7fffU;
}

void printDepthGrid(const uint8_t *buffer)
{
  const uint16_t outputPixels =
      static_cast<uint16_t>(kRangingOutputWidth) * static_cast<uint16_t>(kRangingOutputHeight);

  if (outputPixels > 64U) {
    Serial.println("Depth grid omitted; frame is larger than 64 output pixels.");
    return;
  }

  Serial.println("Depth grid, raw depth units:");
  for (uint8_t y = 0; y < kRangingOutputHeight; ++y) {
    Serial.print("  ");
    for (uint8_t x = 0; x < kRangingOutputWidth; ++x) {
      printPaddedDepth(depthAtOutputPixel(buffer, x, y));
      if (x + 1U < kRangingOutputWidth) {
        Serial.print(' ');
      }
    }
    Serial.println();
  }
}

void printFrameSummary(const uint8_t *buffer, uint16_t bufferSize, uint16_t frameIndex)
{
  const uint32_t amplitudeOffset = static_cast<uint32_t>(kRangingRawResolution) * 2U;
  const uint32_t ambientOffset = amplitudeOffset + (static_cast<uint32_t>(kRangingRawResolution) * 2U);
  const uint8_t centerX = kRangingOutputWidth / 2U;
  const uint8_t centerY = kRangingOutputHeight / 2U;
  const uint16_t centerRawIndex =
      static_cast<uint16_t>(centerY + kRangingOutputYOffset) * kRangingRawWidth + centerX;

  uint16_t minDepth = 0xffffU;
  uint16_t maxDepth = 0U;
  uint32_t sumDepth = 0U;
  uint16_t nonzeroCount = 0U;
  uint16_t flaggedCount = 0U;

  for (uint16_t i = 0; i < kRangingRawResolution; ++i) {
    const uint16_t rawDepth = readLe16(&buffer[i * 2U]);
    const uint16_t depth = rawDepth & 0x7fffU;
    if ((rawDepth & 0x8000U) != 0U) {
      ++flaggedCount;
    }
    if (depth == 0U) {
      continue;
    }
    minDepth = min(minDepth, depth);
    maxDepth = max(maxDepth, depth);
    sumDepth += depth;
    ++nonzeroCount;
  }

  const uint16_t centerDepth = readLe16(&buffer[centerRawIndex * 2U]) & 0x7fffU;
  const uint16_t centerAmplitude = readLe16(&buffer[amplitudeOffset + (centerRawIndex * 2U)]);
  const uint16_t centerAmbient = readLe16(&buffer[ambientOffset + (centerRawIndex * 2U)]);
  const uint8_t *statusLine = &buffer[bufferSize - VL53L9_STATUS_SIZE];
  const uint32_t frameCounter = readLe32(statusLine);
  const uint16_t temperature = readLe16(statusLine + 4U);

  Serial.print("Frame ");
  Serial.print(frameIndex);
  Serial.print(": sensor counter ");
  Serial.print(frameCounter);
  Serial.print(", raw bytes ");
  Serial.print(bufferSize);
  Serial.print(", nonzero depth pixels ");
  Serial.print(nonzeroCount);
  Serial.print('/');
  Serial.print(kRangingRawResolution);
  Serial.print(", flagged ");
  Serial.println(flaggedCount);

  Serial.print("Depth summary: min ");
  Serial.print(nonzeroCount == 0U ? 0U : minDepth);
  Serial.print(", avg ");
  Serial.print(nonzeroCount == 0U ? 0U : (sumDepth / nonzeroCount));
  Serial.print(", max ");
  Serial.print(maxDepth);
  Serial.print(", center ");
  Serial.print(centerDepth);
  Serial.print(", center amp ");
  Serial.print(centerAmplitude);
  Serial.print(", center ambient ");
  Serial.print(centerAmbient);
  Serial.print(", status temp raw ");
  Serial.println(temperature);

  printDepthGrid(buffer);
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
bool configureRanging(vl53l9_arduino_device_t *device, uint16_t *rawBufferSize)
{
  int status = vl53l9_get_raw_buffer_size(kRangingBinning, rawBufferSize);
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
  Serial.print(syncModeName(kRangingSyncMode));
  Serial.print(", power ");
  Serial.print(powerModeName(kRangingPowerMode));
  Serial.print(", context ");
  Serial.print(contextName(kRangingContext));
  Serial.print(", binning ");
  Serial.print(kRangingBinning);
  Serial.print(", output ");
  Serial.print(kRangingOutputWidth);
  Serial.print('x');
  Serial.print(kRangingOutputHeight);
  Serial.print(", raw ");
  Serial.print(kRangingRawWidth);
  Serial.print('x');
  Serial.print(kRangingRawResolution / kRangingRawWidth);
  Serial.print(", frame period ");
  Serial.print(kRangingFramePeriodUs);
  Serial.print(" us, exposure ");
  Serial.print(kRangingExposureMs);
  Serial.print(" ms, buffer ");
  Serial.print(*rawBufferSize);
  Serial.println(" bytes");

  status = vl53l9_set_sync_mode(device, kRangingSyncMode);
  if (!printVl53Status("vl53l9_set_sync_mode", status)) {
    return false;
  }
  status = vl53l9_set_power_mode(device, kRangingPowerMode);
  if (!printVl53Status("vl53l9_set_power_mode", status)) {
    return false;
  }
  status = vl53l9_set_frame_period(device, kRangingFramePeriodUs);
  if (!printVl53Status("vl53l9_set_frame_period", status)) {
    return false;
  }
  status = vl53l9_set_context(device, kRangingContext);
  if (!printVl53Status("vl53l9_set_context", status)) {
    return false;
  }
  status = vl53l9_set_binning(device, kRangingContext, kRangingBinning);
  if (!printVl53Status("vl53l9_set_binning", status)) {
    return false;
  }
  status = vl53l9_set_exposure(device, kRangingContext, kRangingExposureMs);
  if (!printVl53Status("vl53l9_set_exposure", status)) {
    return false;
  }

  return true;
}

bool waitForRangingFrame(vl53l9_arduino_device_t *device, uint16_t frameIndex)
{
  const uint32_t startMs = millis();
  uint8_t ready = 0U;

  while (millis() - startMs < kRangingFrameTimeoutMs) {
    const int status = vl53l9_poll_frame(device, &ready);
    if (status != VL53L9_ERROR_NONE) {
      printVl53Status("vl53l9_poll_frame", status);
      return false;
    }
    if (ready != 0U) {
      Serial.print("Frame ");
      Serial.print(frameIndex);
      Serial.print(" ready after ");
      Serial.print(millis() - startMs);
      Serial.print(" ms, HOST_INTR=");
      Serial.println(digitalRead(kHostIntrPin));
      return true;
    }
    delay(5);
  }

  Serial.print("Timed out waiting for frame ");
  Serial.print(frameIndex);
  Serial.print(" after ");
  Serial.print(kRangingFrameTimeoutMs);
  Serial.print(" ms, HOST_INTR=");
  Serial.println(digitalRead(kHostIntrPin));
  return false;
}

bool runStRangingSample(vl53l9_arduino_device_t *device)
{
  if (!kEnableRangingSample) {
    Serial.println("Ranging sample disabled.");
    return true;
  }

  uint16_t rawBufferSize = 0U;
  if (!configureRanging(device, &rawBufferSize)) {
    return false;
  }

  if (kRangingSampleCount == 0U) {
    Serial.println("Ranging sample count is zero; skipping stream start.");
    return true;
  }

  int status = vl53l9_start(device);
  if (!printVl53Status("vl53l9_start", status)) {
    return false;
  }

  bool ok = true;
  for (uint16_t frameIndex = 1; frameIndex <= kRangingSampleCount; ++frameIndex) {
    if (kRangingSyncMode == VL53L9_SYNC_MANUAL) {
      status = vl53l9_trigger_frame(device);
      if (!printVl53Status("vl53l9_trigger_frame", status)) {
        ok = false;
        break;
      }
    }

    if (!waitForRangingFrame(device, frameIndex)) {
      ok = false;
      break;
    }

    status = vl53l9_get_frame(device, gFrameBuffer, rawBufferSize);
    if (!printVl53Status("vl53l9_get_frame", status)) {
      ok = false;
      break;
    }

    printFrameSummary(gFrameBuffer, rawBufferSize, frameIndex);
  }

  status = vl53l9_stop(device);
  if (!printVl53Status("vl53l9_stop", status)) {
    ok = false;
  }

  return ok;
}

bool runStDriverInit()
{
  vl53l9_arduino_device_init(&gVl53Device, kI3cDynamicAddress);
  vl53l9_arduino_device_set_power_config(&gVl53Device, VDDA_2V8, VDDIO_1V8, kHostClkInHz);

  Serial.println("Starting ST vl53l9_init()");
  vl53l9_arduino_clear_last_error();
  const int initStatus = vl53l9_init(&gVl53Device);
  if (!printVl53Status("vl53l9_init", initStatus)) {
    printVl53PlatformError();
    return false;
  }

  uint32_t deviceId = 0;
  const int idStatus = vl53l9_get_device_id(&gVl53Device, &deviceId);
  Serial.print("vl53l9_get_device_id: ");
  Serial.print(idStatus);
  Serial.print(" (");
  Serial.print(vl53l9ErrorName(idStatus));
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
  if (!runStRangingSample(&gVl53Device)) {
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
