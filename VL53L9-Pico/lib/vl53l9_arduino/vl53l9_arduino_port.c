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

#ifndef VL53L9_ARDUINO_I3C_IBI_RETRIES
#define VL53L9_ARDUINO_I3C_IBI_RETRIES 3U
#endif

#ifndef VL53L9_ARDUINO_I3C_IBI_PAYLOAD_SIZE
#define VL53L9_ARDUINO_I3C_IBI_PAYLOAD_SIZE 16U
#endif

#if (VL53L9_ARDUINO_I3C_WRITE_CHUNK_SIZE < 3U)
#error "VL53L9_ARDUINO_I3C_WRITE_CHUNK_SIZE must be at least 3"
#endif

#if (VL53L9_ARDUINO_I3C_READ_CHUNK_SIZE < 1U)
#error "VL53L9_ARDUINO_I3C_READ_CHUNK_SIZE must be at least 1"
#endif

static vl53l9_arduino_platform_error_t g_last_error;

static int i3c_status_to_vl53l9_error(i3c_hl_status_t status)
{
    return (status == i3c_hl_status_ok) ? VL53L9_ERROR_NONE : VL53L9_ERROR_PLATFORM;
}

static vl53l9_arduino_device_t *checked_device(void *const p_dev)
{
    return (vl53l9_arduino_device_t *)p_dev;
}

static void record_i3c_error(const char *operation,
                             uint8_t i3c_address,
                             uint16_t register_address,
                             uint32_t requested_size,
                             uint32_t actual_size,
                             uint32_t chunk_offset,
                             uint32_t ibi_retries,
                             i3c_hl_status_t status)
{
    g_last_error.valid = 1U;
    g_last_error.operation = operation;
    g_last_error.i3c_address = i3c_address;
    g_last_error.register_address = register_address;
    g_last_error.requested_size = requested_size;
    g_last_error.actual_size = actual_size;
    g_last_error.chunk_offset = chunk_offset;
    g_last_error.ibi_retries = ibi_retries;
    g_last_error.i3c_status = (int)status;
}

static i3c_hl_status_t drain_pending_ibi(void)
{
    uint8_t payload[VL53L9_ARDUINO_I3C_IBI_PAYLOAD_SIZE];
    uint32_t length = sizeof(payload);
    i3c_hl_status_t status = i3c_hl_poll(payload, &length);

    return (status == i3c_hl_status_no_ibi) ? i3c_hl_status_ok : status;
}

static i3c_hl_status_t privwrite_with_ibi_retry(uint8_t i3c_address,
                                                const uint8_t *data,
                                                uint32_t size,
                                                uint32_t *ibi_retries)
{
    i3c_hl_status_t status = i3c_hl_status_ok;
    uint32_t retries = 0U;

    while (1)
    {
        status = i3c_hl_sdr_privwrite(i3c_address, data, size);
        if (status != i3c_hl_status_ibi)
        {
            break;
        }
        status = drain_pending_ibi();
        if (status != i3c_hl_status_ok)
        {
            break;
        }
        if (retries >= VL53L9_ARDUINO_I3C_IBI_RETRIES)
        {
            status = i3c_hl_status_ibi;
            break;
        }
        retries++;
    }

    if (ibi_retries != NULL)
    {
        *ibi_retries = retries;
    }
    return status;
}

static i3c_hl_status_t privwriteread_with_ibi_retry(uint8_t i3c_address,
                                                    const uint8_t *write_data,
                                                    uint32_t write_size,
                                                    uint8_t *read_data,
                                                    uint32_t requested_read_size,
                                                    uint32_t *actual_read_size,
                                                    uint32_t *ibi_retries)
{
    i3c_hl_status_t status = i3c_hl_status_ok;
    uint32_t retries = 0U;

    while (1)
    {
        *actual_read_size = requested_read_size;
        status = i3c_hl_sdr_privwriteread(i3c_address,
                                          write_data,
                                          write_size,
                                          read_data,
                                          actual_read_size);
        if (status != i3c_hl_status_ibi)
        {
            break;
        }
        status = drain_pending_ibi();
        if (status != i3c_hl_status_ok)
        {
            break;
        }
        if (retries >= VL53L9_ARDUINO_I3C_IBI_RETRIES)
        {
            status = i3c_hl_status_ibi;
            break;
        }
        retries++;
    }

    if (ibi_retries != NULL)
    {
        *ibi_retries = retries;
    }
    return status;
}

void vl53l9_arduino_clear_last_error(void)
{
    memset(&g_last_error, 0, sizeof(g_last_error));
}

const vl53l9_arduino_platform_error_t *vl53l9_arduino_get_last_error(void)
{
    return &g_last_error;
}

const char *vl53l9_arduino_i3c_status_name(int status)
{
    return i3c_hl_get_errorstring((i3c_hl_status_t)status);
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
        uint32_t ibi_retries = 0U;
        i3c_hl_status_t status;

        if (addr32 > 0xffffU)
        {
            return VL53L9_ERROR_INVALID_PARAM;
        }

        command[0] = (uint8_t)((addr32 >> 8) & 0xffU);
        command[1] = (uint8_t)(addr32 & 0xffU);
        status = privwriteread_with_ibi_retry(device->i3c_address,
                                              command,
                                              sizeof(command),
                                              &p_values[offset],
                                              this_read,
                                              &read_size,
                                              &ibi_retries);
        if (status != i3c_hl_status_ok)
        {
            record_i3c_error("read",
                             device->i3c_address,
                             (uint16_t)addr32,
                             this_read,
                             read_size,
                             offset,
                             ibi_retries,
                             status);
            return i3c_status_to_vl53l9_error(status);
        }
        if (read_size != this_read)
        {
            record_i3c_error("read-short",
                             device->i3c_address,
                             (uint16_t)addr32,
                             this_read,
                             read_size,
                             offset,
                             ibi_retries,
                             status);
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
        uint32_t ibi_retries = 0U;
        i3c_hl_status_t status;

        if (addr32 > 0xffffU)
        {
            return VL53L9_ERROR_INVALID_PARAM;
        }

        data_write[0] = (uint8_t)((addr32 >> 8) & 0xffU);
        data_write[1] = (uint8_t)(addr32 & 0xffU);
        memcpy(&data_write[2], &p_values[offset], payload);

        status = privwrite_with_ibi_retry(device->i3c_address, data_write, write_size, &ibi_retries);
        if (status != i3c_hl_status_ok)
        {
            record_i3c_error("write",
                             device->i3c_address,
                             (uint16_t)addr32,
                             write_size,
                             write_size,
                             offset,
                             ibi_retries,
                             status);
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
