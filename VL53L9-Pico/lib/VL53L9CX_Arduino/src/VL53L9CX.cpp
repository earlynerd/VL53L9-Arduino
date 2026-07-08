#include "VL53L9CX.h"

namespace
{
void printHex8(Print &out, uint8_t value)
{
    if (value < 0x10U)
    {
        out.print('0');
    }
    out.print(value, HEX);
}

void printHex16(Print &out, uint16_t value)
{
    out.print("0x");
    printHex8(out, static_cast<uint8_t>(value >> 8));
    printHex8(out, static_cast<uint8_t>(value));
}

void printPaddedDepth(Print &out, uint16_t value)
{
    if (value < 10000U)
    {
        out.print(' ');
    }
    if (value < 1000U)
    {
        out.print(' ');
    }
    if (value < 100U)
    {
        out.print(' ');
    }
    if (value < 10U)
    {
        out.print(' ');
    }
    out.print(value);
}
} // namespace

VL53L9CX::VL53L9CX() : interrupt_pin_(-1)
{
    vl53l9_arduino_device_init(&device_, 0x52U);
}

void VL53L9CX::setAddress(uint8_t dynamic_address)
{
    vl53l9_arduino_device_init(&device_, dynamic_address);
}

void VL53L9CX::setPowerConfig(vl53l9_vdda_t vdda, vl53l9_vddio_t vddio, uint32_t ext_clock_hz)
{
    vl53l9_arduino_device_set_power_config(&device_, vdda, vddio, ext_clock_hz);
}

void VL53L9CX::setInterruptPin(int8_t interrupt_pin)
{
    interrupt_pin_ = interrupt_pin;
}

int VL53L9CX::init()
{
    vl53l9_arduino_clear_last_error();
    return vl53l9_init(&device_);
}

int VL53L9CX::getDeviceId(uint32_t *device_id)
{
    return vl53l9_get_device_id(&device_, device_id);
}

int VL53L9CX::getRawBufferSize(uint8_t binning, uint16_t *size) const
{
    return vl53l9_get_raw_buffer_size(binning, size);
}

int VL53L9CX::configureRanging(const VL53L9CXRangingConfig &config)
{
    int status = setSyncMode(config.sync_mode);
    if (status != VL53L9_ERROR_NONE)
    {
        return status;
    }

    status = setPowerMode(config.power_mode);
    if (status != VL53L9_ERROR_NONE)
    {
        return status;
    }

    status = setFramePeriod(config.frame_period_us);
    if (status != VL53L9_ERROR_NONE)
    {
        return status;
    }

    status = setContext(config.context);
    if (status != VL53L9_ERROR_NONE)
    {
        return status;
    }

    status = setBinning(config.context, config.binning);
    if (status != VL53L9_ERROR_NONE)
    {
        return status;
    }

    return setExposure(config.context, config.exposure_ms);
}

int VL53L9CX::setSyncMode(vl53l9_sync_mode_t mode)
{
    return vl53l9_set_sync_mode(&device_, mode);
}

int VL53L9CX::setPowerMode(vl53l9_power_mode_t mode)
{
    return vl53l9_set_power_mode(&device_, mode);
}

int VL53L9CX::setFramePeriod(uint32_t frame_period_us)
{
    return vl53l9_set_frame_period(&device_, frame_period_us);
}

int VL53L9CX::setContext(vl53l9_context_t context)
{
    return vl53l9_set_context(&device_, context);
}

int VL53L9CX::setBinning(vl53l9_context_t context, uint8_t binning)
{
    return vl53l9_set_binning(&device_, context, binning);
}

int VL53L9CX::setExposure(vl53l9_context_t context, uint16_t exposure_ms)
{
    return vl53l9_set_exposure(&device_, context, exposure_ms);
}

int VL53L9CX::start()
{
    return vl53l9_start(&device_);
}

int VL53L9CX::stop()
{
    return vl53l9_stop(&device_);
}

int VL53L9CX::triggerFrame()
{
    return vl53l9_trigger_frame(&device_);
}

int VL53L9CX::pollFrame(bool *ready)
{
    uint8_t ready_u8 = 0U;
    int status;

    if (ready == nullptr)
    {
        return VL53L9_ERROR_INVALID_PARAM;
    }

    status = vl53l9_poll_frame(&device_, &ready_u8);
    if (status == VL53L9_ERROR_NONE)
    {
        *ready = ready_u8 != 0U;
    }
    return status;
}

int VL53L9CX::waitForFrame(uint32_t timeout_ms,
                           VL53L9CXFrameWaitResult *result,
                           VL53L9CXFrameWaitMode mode,
                           uint16_t poll_interval_ms)
{
    const uint32_t start_ms = millis();
    uint32_t last_poll_ms = start_ms - static_cast<uint32_t>(poll_interval_ms);
    VL53L9CXFrameWaitResult local_result;

    if (poll_interval_ms == 0U)
    {
        poll_interval_ms = 1U;
    }

    while ((millis() - start_ms) < timeout_ms)
    {
        const uint32_t now = millis();
        int interrupt_level = -1;
        const bool gpio_active = interruptIsActive(mode, &interrupt_level);
        const bool poll_due = (now - last_poll_ms) >= poll_interval_ms;

        local_result.interrupt_level = interrupt_level;
        local_result.interrupt_observed_active = local_result.interrupt_observed_active || gpio_active;

        if ((mode == VL53L9CX_FRAME_WAIT_POLL) || gpio_active || poll_due)
        {
            bool ready = false;
            const int status = pollFrame(&ready);
            local_result.polls++;
            last_poll_ms = now;
            if (status != VL53L9_ERROR_NONE)
            {
                local_result.elapsed_ms = now - start_ms;
                local_result.ready = false;
                if (result != nullptr)
                {
                    *result = local_result;
                }
                return status;
            }
            if (ready)
            {
                local_result.elapsed_ms = now - start_ms;
                local_result.ready = true;
                if (result != nullptr)
                {
                    *result = local_result;
                }
                return VL53L9_ERROR_NONE;
            }
        }

        delay(1);
    }

    local_result.elapsed_ms = millis() - start_ms;
    local_result.ready = false;
    if (result != nullptr)
    {
        *result = local_result;
    }
    return VL53L9_ERROR_TIMEOUT;
}

int VL53L9CX::readFrame(uint8_t *buffer, uint16_t size)
{
    return vl53l9_get_frame(&device_, buffer, size);
}

int VL53L9CX::readFrameAfterWait(uint8_t *buffer,
                                 uint16_t size,
                                 uint32_t timeout_ms,
                                 VL53L9CXFrameWaitResult *result,
                                 VL53L9CXFrameWaitMode mode,
                                 uint16_t poll_interval_ms)
{
    const int wait_status = waitForFrame(timeout_ms, result, mode, poll_interval_ms);
    if (wait_status != VL53L9_ERROR_NONE)
    {
        return wait_status;
    }
    return readFrame(buffer, size);
}

int VL53L9CX::ackFrame()
{
    return vl53l9_get_frame(&device_, nullptr, 0U);
}

vl53l9_arduino_device_t *VL53L9CX::device()
{
    return &device_;
}

const vl53l9_arduino_platform_error_t *VL53L9CX::lastPlatformError() const
{
    return vl53l9_arduino_get_last_error();
}

const char *VL53L9CX::errorName(int status)
{
    switch (status)
    {
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

const char *VL53L9CX::syncModeName(vl53l9_sync_mode_t mode)
{
    switch (mode)
    {
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

const char *VL53L9CX::powerModeName(vl53l9_power_mode_t mode)
{
    switch (mode)
    {
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

const char *VL53L9CX::contextName(vl53l9_context_t context)
{
    switch (context)
    {
        case VL53L9_CONTEXT_SHORT:
            return "short";
        case VL53L9_CONTEXT_LONG:
            return "long";
        default:
            return "unknown";
    }
}

const char *VL53L9CX::frameWaitModeName(VL53L9CXFrameWaitMode mode)
{
    switch (mode)
    {
        case VL53L9CX_FRAME_WAIT_POLL:
            return "poll";
        case VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_LOW:
            return "gpio-active-low";
        case VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_HIGH:
            return "gpio-active-high";
        default:
            return "unknown";
    }
}

void VL53L9CX::printPlatformError(Print &out, const vl53l9_arduino_platform_error_t *error)
{
    if ((error == nullptr) || (error->valid == 0U))
    {
        out.println("VL53L9 platform error detail: no I3C failure recorded");
        return;
    }

    out.print("VL53L9 platform error detail: op=");
    out.print(error->operation != nullptr ? error->operation : "unknown");
    out.print(", addr=");
    printHex8(out, error->i3c_address);
    out.print(", reg=");
    printHex16(out, error->register_address);
    out.print(", requested=");
    out.print(error->requested_size);
    out.print(", actual=");
    out.print(error->actual_size);
    out.print(", chunk_offset=");
    out.print(error->chunk_offset);
    out.print(", ibi_retries=");
    out.print(error->ibi_retries);
    out.print(", i3c_status=");
    out.print(error->i3c_status);
    out.print(" (");
    out.print(vl53l9_arduino_i3c_status_name(error->i3c_status));
    out.println(')');
}

uint16_t VL53L9CX::readLe16(const uint8_t *data)
{
    if (data == nullptr)
    {
        return 0U;
    }
    return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t VL53L9CX::readLe32(const uint8_t *data)
{
    if (data == nullptr)
    {
        return 0U;
    }
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

uint16_t VL53L9CX::depthAtOutputPixel(const uint8_t *buffer, uint8_t binning, uint8_t x, uint8_t y)
{
    const uint8_t raw_width = rawWidthForBinning(binning);
    const uint8_t output_width = outputWidthForBinning(binning);
    const uint8_t output_height = outputHeightForBinning(binning);
    const uint8_t output_y_offset = outputYOffsetForBinning(binning);

    if ((buffer == nullptr) || (raw_width == 0U) || (x >= output_width) || (y >= output_height))
    {
        return 0U;
    }

    const uint16_t raw_index = static_cast<uint16_t>(y + output_y_offset) * raw_width + x;
    return readLe16(&buffer[raw_index * 2U]) & 0x7fffU;
}

bool VL53L9CX::summarizeRawFrame(const uint8_t *buffer,
                                 uint16_t size,
                                 uint8_t binning,
                                 VL53L9CXRawFrameSummary *summary)
{
    const uint16_t raw_resolution = rawResolutionForBinning(binning);
    const uint16_t expected_size = rawBufferSizeForBinning(binning);
    const uint8_t raw_width = rawWidthForBinning(binning);
    const uint8_t output_width = outputWidthForBinning(binning);
    const uint8_t output_height = outputHeightForBinning(binning);
    const uint8_t output_y_offset = outputYOffsetForBinning(binning);

    if ((buffer == nullptr) || (summary == nullptr) || (raw_resolution == 0U) ||
        (expected_size == 0U) || (size < expected_size) || (raw_width == 0U) ||
        (output_width == 0U) || (output_height == 0U))
    {
        return false;
    }

    const uint32_t amplitude_offset = static_cast<uint32_t>(raw_resolution) * 2U;
    const uint32_t ambient_offset = amplitude_offset + (static_cast<uint32_t>(raw_resolution) * 2U);
    const uint8_t center_x = output_width / 2U;
    const uint8_t center_y = output_height / 2U;
    const uint16_t center_raw_index = static_cast<uint16_t>(center_y + output_y_offset) * raw_width + center_x;
    const uint8_t *status_line = &buffer[size - VL53L9_STATUS_SIZE];

    uint16_t min_depth = 0xffffU;
    uint16_t max_depth = 0U;
    uint32_t sum_depth = 0U;
    uint16_t nonzero_count = 0U;
    uint16_t flagged_count = 0U;

    for (uint16_t i = 0U; i < raw_resolution; ++i)
    {
        const uint16_t raw_depth = readLe16(&buffer[i * 2U]);
        const uint16_t depth = raw_depth & 0x7fffU;
        if ((raw_depth & 0x8000U) != 0U)
        {
            flagged_count++;
        }
        if (depth == 0U)
        {
            continue;
        }
        if (depth < min_depth)
        {
            min_depth = depth;
        }
        if (depth > max_depth)
        {
            max_depth = depth;
        }
        sum_depth += depth;
        nonzero_count++;
    }

    summary->raw_size = size;
    summary->raw_resolution = raw_resolution;
    summary->nonzero_depth_pixels = nonzero_count;
    summary->flagged_pixels = flagged_count;
    summary->min_depth = nonzero_count == 0U ? 0U : min_depth;
    summary->average_depth = nonzero_count == 0U ? 0U : static_cast<uint16_t>(sum_depth / nonzero_count);
    summary->max_depth = max_depth;
    summary->center_depth = readLe16(&buffer[center_raw_index * 2U]) & 0x7fffU;
    summary->center_amplitude = readLe16(&buffer[amplitude_offset + (center_raw_index * 2U)]);
    summary->center_ambient = readLe16(&buffer[ambient_offset + (center_raw_index * 2U)]);
    summary->frame_counter = readLe32(status_line);
    summary->temperature_raw = readLe16(status_line + 4U);
    return true;
}

bool VL53L9CX::printRawDepthGrid(Print &out, const uint8_t *buffer, uint8_t binning, uint16_t max_output_pixels)
{
    const uint8_t output_width = outputWidthForBinning(binning);
    const uint8_t output_height = outputHeightForBinning(binning);
    const uint16_t output_pixels = static_cast<uint16_t>(output_width) * static_cast<uint16_t>(output_height);

    if ((buffer == nullptr) || (output_width == 0U) || (output_height == 0U))
    {
        return false;
    }
    if (output_pixels > max_output_pixels)
    {
        out.println("Depth grid omitted; frame is larger than display limit.");
        return true;
    }

    out.println("Depth grid, raw depth units:");
    for (uint8_t y = 0U; y < output_height; ++y)
    {
        out.print("  ");
        for (uint8_t x = 0U; x < output_width; ++x)
        {
            printPaddedDepth(out, depthAtOutputPixel(buffer, binning, x, y));
            if (x + 1U < output_width)
            {
                out.print(' ');
            }
        }
        out.println();
    }

    return true;
}

bool VL53L9CX::printRawFrameSummary(Print &out,
                                    const uint8_t *buffer,
                                    uint16_t size,
                                    uint16_t frame_index,
                                    uint8_t binning)
{
    VL53L9CXRawFrameSummary summary;
    if (!summarizeRawFrame(buffer, size, binning, &summary))
    {
        out.println("Raw frame summary failed; frame buffer shape is invalid.");
        return false;
    }

    out.print("Frame ");
    out.print(frame_index);
    out.print(": sensor counter ");
    out.print(summary.frame_counter);
    out.print(", raw bytes ");
    out.print(summary.raw_size);
    out.print(", nonzero depth pixels ");
    out.print(summary.nonzero_depth_pixels);
    out.print('/');
    out.print(summary.raw_resolution);
    out.print(", flagged ");
    out.println(summary.flagged_pixels);

    out.print("Depth summary: min ");
    out.print(summary.min_depth);
    out.print(", avg ");
    out.print(summary.average_depth);
    out.print(", max ");
    out.print(summary.max_depth);
    out.print(", center ");
    out.print(summary.center_depth);
    out.print(", center amp ");
    out.print(summary.center_amplitude);
    out.print(", center ambient ");
    out.print(summary.center_ambient);
    out.print(", status temp raw ");
    out.println(summary.temperature_raw);

    return printRawDepthGrid(out, buffer, binning);
}

bool VL53L9CX::interruptIsActive(VL53L9CXFrameWaitMode mode, int *level) const
{
    if ((mode == VL53L9CX_FRAME_WAIT_POLL) || (interrupt_pin_ < 0))
    {
        if (level != nullptr)
        {
            *level = -1;
        }
        return false;
    }

    const int pin_level = digitalRead(static_cast<uint8_t>(interrupt_pin_));
    if (level != nullptr)
    {
        *level = pin_level;
    }

    if (mode == VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_LOW)
    {
        return pin_level == LOW;
    }
    if (mode == VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_HIGH)
    {
        return pin_level == HIGH;
    }
    return false;
}
