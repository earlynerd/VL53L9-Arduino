#ifndef VL53L9_ARDUINO_PORT_H
#define VL53L9_ARDUINO_PORT_H

#include <stdint.h>
#include "vl53l9.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t i3c_address;
    vl53l9_vdda_t vdda;
    vl53l9_vddio_t vddio;
    uint32_t ext_clock_hz;
} vl53l9_arduino_device_t;

void vl53l9_arduino_device_init(vl53l9_arduino_device_t *device, uint8_t i3c_address);
void vl53l9_arduino_device_set_power_config(vl53l9_arduino_device_t *device,
                                            vl53l9_vdda_t vdda,
                                            vl53l9_vddio_t vddio,
                                            uint32_t ext_clock_hz);

#ifdef __cplusplus
}
#endif

#endif // VL53L9_ARDUINO_PORT_H
