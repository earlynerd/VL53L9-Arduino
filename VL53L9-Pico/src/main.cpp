#include <Arduino.h>
#include "i3c_hl.h"

#ifndef VL53L9_I3C_SDA_PIN
#define VL53L9_I3C_SDA_PIN 16
#endif

#ifndef VL53L9_I3C_SCL_PIN
#define VL53L9_I3C_SCL_PIN 17
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

namespace {
constexpr uint8_t kI3cSdaPin = VL53L9_I3C_SDA_PIN;
constexpr uint8_t kI3cSclPin = VL53L9_I3C_SCL_PIN;
constexpr uint8_t kI3cPioIndex = VL53L9_I3C_PIO_INDEX;
constexpr uint8_t kI3cStateMachine = VL53L9_I3C_SM;
constexpr uint8_t kI3cDriveStrengthMa = VL53L9_I3C_DRIVE_STRENGTH_MA;
constexpr uint32_t kI3cClockKhz = VL53L9_I3C_CLK_KHZ;
constexpr bool kI3cDisableInterrupts = VL53L9_I3C_DISABLE_INTERRUPTS != 0;

static_assert(kI3cSclPin == kI3cSdaPin + 1, "I3C PIO requires SCL = SDA + 1");

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
    Serial.print("I3C init failed: ");
    Serial.println(i3c_hl_get_errorstring(status));
    return false;
  }

  status = i3c_hl_set_clkrate(kI3cClockKhz);
  if (status != i3c_hl_status_ok) {
    Serial.print("I3C clock setup failed: ");
    Serial.println(i3c_hl_get_errorstring(status));
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
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("VL53L9CX RP2350 I3C transport bring-up");
  initI3cTransport();
}

void loop() {
}
