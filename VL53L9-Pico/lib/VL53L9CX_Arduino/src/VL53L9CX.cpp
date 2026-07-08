#include "VL53L9CX.h"

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
