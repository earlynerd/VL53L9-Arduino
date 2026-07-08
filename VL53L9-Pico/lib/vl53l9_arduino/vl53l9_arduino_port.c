#include "vl53l9_arduino_port.h"

#include <stddef.h>
#include <string.h>
#include "i3c_hl.h"
#include "pico/stdlib.h"
#include "vl53l9_platform.h"

#ifndef VL53L9_ARDUINO_I3C_WRITE_CHUNK_SIZE
#define VL53L9_ARDUINO_I3C_WRITE_CHUNK_SIZE 64U
#endif

#ifndef VL53L9_ARDUINO_I3C_READ_CHUNK_SIZE
#define VL53L9_ARDUINO_I3C_READ_CHUNK_SIZE 256U
#endif

#if (VL53L9_ARDUINO_I3C_WRITE_CHUNK_SIZE < 3U)
#error "VL53L9_ARDUINO_I3C_WRITE_CHUNK_SIZE must be at least 3"
#endif

#if (VL53L9_ARDUINO_I3C_READ_CHUNK_SIZE < 1U)
#error "VL53L9_ARDUINO_I3C_READ_CHUNK_SIZE must be at least 1"
#endif

static int i3c_status_to_vl53l9_error(i3c_hl_status_t status)
{
    return (status == i3c_hl_status_ok) ? VL53L9_ERROR_NONE : VL53L9_ERROR_PLATFORM;
}

static vl53l9_arduino_device_t *checked_device(void *const p_dev)
{
    return (vl53l9_arduino_device_t *)p_dev;
}

void vl53l9_arduino_device_init(vl53l9_arduino_device_t *device, uint8_t i3c_address)
{
    if (device == NULL)
    {
        return;
    }

    device->i3c_address = i3c_address & 0x7fU;
    device->vdda = VDDA_2V8;
    device->vddio = VDDIO_1V8;
    device->ext_clock_hz = 12000000U;
}

void vl53l9_arduino_device_set_power_config(vl53l9_arduino_device_t *device,
                                            vl53l9_vdda_t vdda,
                                            vl53l9_vddio_t vddio,
                                            uint32_t ext_clock_hz)
{
    if (device == NULL)
    {
        return;
    }

    device->vdda = vdda;
    device->vddio = vddio;
    device->ext_clock_hz = ext_clock_hz;
}

int vl53l9_read(void *const p_dev, uint16_t address, uint8_t *p_values, uint32_t size)
{
    vl53l9_arduino_device_t *device = checked_device(p_dev);
    uint32_t offset = 0U;

    if ((device == NULL) || (p_values == NULL))
    {
        return VL53L9_ERROR_INVALID_PARAM;
    }
    if (size == 0U)
    {
        return VL53L9_ERROR_NONE;
    }

    while (offset < size)
    {
        const uint32_t remaining = size - offset;
        const uint32_t this_read = (remaining < VL53L9_ARDUINO_I3C_READ_CHUNK_SIZE)
                                       ? remaining
                                       : VL53L9_ARDUINO_I3C_READ_CHUNK_SIZE;
        const uint32_t addr32 = (uint32_t)address + offset;
        uint8_t command[2];
        uint32_t read_size = this_read;
        i3c_hl_status_t status;

        if (addr32 > 0xffffU)
        {
            return VL53L9_ERROR_INVALID_PARAM;
        }

        command[0] = (uint8_t)((addr32 >> 8) & 0xffU);
        command[1] = (uint8_t)(addr32 & 0xffU);
        status = i3c_hl_sdr_privwriteread(device->i3c_address,
                                          command,
                                          sizeof(command),
                                          &p_values[offset],
                                          &read_size);
        if (status != i3c_hl_status_ok)
        {
            return i3c_status_to_vl53l9_error(status);
        }
        if (read_size != this_read)
        {
            return VL53L9_ERROR_PLATFORM;
        }

        offset += this_read;
    }

    return VL53L9_ERROR_NONE;
}

int vl53l9_read8(void *const p_dev, uint16_t address, uint8_t *p_value)
{
    return vl53l9_read(p_dev, address, p_value, 1U);
}

int vl53l9_read16(void *const p_dev, uint16_t address, uint16_t *p_value)
{
    uint8_t data[2];
    int ret;

    if (p_value == NULL)
    {
        return VL53L9_ERROR_INVALID_PARAM;
    }

    ret = vl53l9_read(p_dev, address, data, sizeof(data));
    if (ret != VL53L9_ERROR_NONE)
    {
        return ret;
    }

    *p_value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return VL53L9_ERROR_NONE;
}

int vl53l9_read32(void *const p_dev, uint16_t address, uint32_t *p_value)
{
    uint8_t data[4];
    int ret;

    if (p_value == NULL)
    {
        return VL53L9_ERROR_INVALID_PARAM;
    }

    ret = vl53l9_read(p_dev, address, data, sizeof(data));
    if (ret != VL53L9_ERROR_NONE)
    {
        return ret;
    }

    *p_value = (uint32_t)data[0] |
               ((uint32_t)data[1] << 8) |
               ((uint32_t)data[2] << 16) |
               ((uint32_t)data[3] << 24);
    return VL53L9_ERROR_NONE;
}

int vl53l9_read_async(void *const p_dev, uint16_t address, volatile uint8_t *p_values, uint32_t size)
{
    return vl53l9_read(p_dev, address, (uint8_t *)p_values, size);
}

int vl53l9_write(void *const p_dev, uint16_t address, uint8_t *p_values, uint32_t size)
{
    vl53l9_arduino_device_t *device = checked_device(p_dev);
    uint8_t data_write[VL53L9_ARDUINO_I3C_WRITE_CHUNK_SIZE];
    const uint32_t max_payload = sizeof(data_write) - 2U;
    uint32_t offset = 0U;

    if ((device == NULL) || (p_values == NULL))
    {
        return VL53L9_ERROR_INVALID_PARAM;
    }
    if (size == 0U)
    {
        return VL53L9_ERROR_NONE;
    }

    while (offset < size)
    {
        const uint32_t remaining = size - offset;
        const uint32_t payload = (remaining < max_payload) ? remaining : max_payload;
        const uint32_t addr32 = (uint32_t)address + offset;
        const uint32_t write_size = payload + 2U;
        i3c_hl_status_t status;

        if (addr32 > 0xffffU)
        {
            return VL53L9_ERROR_INVALID_PARAM;
        }

        data_write[0] = (uint8_t)((addr32 >> 8) & 0xffU);
        data_write[1] = (uint8_t)(addr32 & 0xffU);
        memcpy(&data_write[2], &p_values[offset], payload);

        status = i3c_hl_sdr_privwrite(device->i3c_address, data_write, write_size);
        if (status != i3c_hl_status_ok)
        {
            return i3c_status_to_vl53l9_error(status);
        }

        offset += payload;
    }

    return VL53L9_ERROR_NONE;
}

int vl53l9_write8(void *const p_dev, uint16_t address, uint8_t value)
{
    return vl53l9_write(p_dev, address, &value, 1U);
}

int vl53l9_write16(void *const p_dev, uint16_t address, uint16_t value)
{
    uint8_t data[2] = {
        (uint8_t)(value & 0xffU),
        (uint8_t)((value >> 8) & 0xffU),
    };
    return vl53l9_write(p_dev, address, data, sizeof(data));
}

int vl53l9_write32(void *const p_dev, uint16_t address, uint32_t value)
{
    uint8_t data[4] = {
        (uint8_t)(value & 0xffU),
        (uint8_t)((value >> 8) & 0xffU),
        (uint8_t)((value >> 16) & 0xffU),
        (uint8_t)((value >> 24) & 0xffU),
    };
    return vl53l9_write(p_dev, address, data, sizeof(data));
}

int vl53l9_wait_ms(void *const p_dev, uint32_t delay_ms)
{
    (void)p_dev;
    sleep_ms(delay_ms);
    return VL53L9_ERROR_NONE;
}

int vl53l9_get_config_vddio(void *const p_dev, vl53l9_vddio_t *voltage)
{
    vl53l9_arduino_device_t *device = checked_device(p_dev);
    if ((device == NULL) || (voltage == NULL))
    {
        return VL53L9_ERROR_INVALID_PARAM;
    }

    *voltage = device->vddio;
    return VL53L9_ERROR_NONE;
}

int vl53l9_get_config_vdda(void *const p_dev, vl53l9_vdda_t *voltage)
{
    vl53l9_arduino_device_t *device = checked_device(p_dev);
    if ((device == NULL) || (voltage == NULL))
    {
        return VL53L9_ERROR_INVALID_PARAM;
    }

    *voltage = device->vdda;
    return VL53L9_ERROR_NONE;
}

int vl53l9_get_config_ext_clock(void *const p_dev, uint32_t *ext_clock)
{
    vl53l9_arduino_device_t *device = checked_device(p_dev);
    if ((device == NULL) || (ext_clock == NULL))
    {
        return VL53L9_ERROR_INVALID_PARAM;
    }

    *ext_clock = device->ext_clock_hz;
    return VL53L9_ERROR_NONE;
}
