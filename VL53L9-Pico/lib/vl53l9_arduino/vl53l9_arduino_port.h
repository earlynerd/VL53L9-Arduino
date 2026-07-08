#ifndef VL53L9_ARDUINO_PORT_H
#define VL53L9_ARDUINO_PORT_H

#include <stdint.h>
#include "vl53l9.h"

#ifndef VL53L9_ARDUINO_I3C_IBI_PAYLOAD_SIZE
#define VL53L9_ARDUINO_I3C_IBI_PAYLOAD_SIZE 16U
#endif

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

typedef struct
{
    uint8_t valid;
    const char *operation;
    uint8_t i3c_address;
    uint16_t register_address;
    uint32_t requested_size;
    uint32_t actual_size;
    uint32_t chunk_offset;
    uint32_t ibi_retries;
    uint32_t ibi_payload_length;
    uint8_t ibi_payload[VL53L9_ARDUINO_I3C_IBI_PAYLOAD_SIZE];
    int i3c_status;
} vl53l9_arduino_platform_error_t;

void vl53l9_arduino_device_init(vl53l9_arduino_device_t *device, uint8_t i3c_address);
void vl53l9_arduino_device_set_power_config(vl53l9_arduino_device_t *device,
                                            vl53l9_vdda_t vdda,
                                            vl53l9_vddio_t vddio,
                                            uint32_t ext_clock_hz);
void vl53l9_arduino_clear_last_error(void);
const vl53l9_arduino_platform_error_t *vl53l9_arduino_get_last_error(void);
const char *vl53l9_arduino_i3c_status_name(int status);

#ifdef __cplusplus
}
#endif

#endif // VL53L9_ARDUINO_PORT_H
