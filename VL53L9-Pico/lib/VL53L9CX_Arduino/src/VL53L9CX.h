#ifndef VL53L9CX_ARDUINO_H
#define VL53L9CX_ARDUINO_H

#include <Arduino.h>
#include <stdint.h>

#include "vl53l9_arduino_port.h"

enum VL53L9CXFrameWaitMode : uint8_t
{
    VL53L9CX_FRAME_WAIT_POLL = 0,
    VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_LOW = 1,
    VL53L9CX_FRAME_WAIT_GPIO_ACTIVE_HIGH = 2,
};

struct VL53L9CXRangingConfig
{
    vl53l9_sync_mode_t sync_mode = VL53L9_SYNC_AUTONOMOUS;
    vl53l9_power_mode_t power_mode = VL53L9_POWER_REGULAR;
    vl53l9_context_t context = VL53L9_CONTEXT_SHORT;
    uint8_t binning = 12U;
    uint32_t frame_period_us = 250000U;
    uint16_t exposure_ms = 5U;
};

struct VL53L9CXFrameWaitResult
{
    bool ready = false;
    uint32_t elapsed_ms = 0U;
    uint16_t polls = 0U;
    int interrupt_level = -1;
    bool interrupt_observed_active = false;
};

class VL53L9CX
{
public:
    VL53L9CX();

    void setAddress(uint8_t dynamic_address);
    void setPowerConfig(vl53l9_vdda_t vdda, vl53l9_vddio_t vddio, uint32_t ext_clock_hz);
    void setInterruptPin(int8_t interrupt_pin);

    int init();
    int getDeviceId(uint32_t *device_id);

    int getRawBufferSize(uint8_t binning, uint16_t *size) const;
    int configureRanging(const VL53L9CXRangingConfig &config);
    int setSyncMode(vl53l9_sync_mode_t mode);
    int setPowerMode(vl53l9_power_mode_t mode);
    int setFramePeriod(uint32_t frame_period_us);
    int setContext(vl53l9_context_t context);
    int setBinning(vl53l9_context_t context, uint8_t binning);
    int setExposure(vl53l9_context_t context, uint16_t exposure_ms);

    int start();
    int stop();
    int triggerFrame();
    int pollFrame(bool *ready);
    int waitForFrame(uint32_t timeout_ms,
                     VL53L9CXFrameWaitResult *result = nullptr,
                     VL53L9CXFrameWaitMode mode = VL53L9CX_FRAME_WAIT_POLL,
                     uint16_t poll_interval_ms = 5U);
    int readFrame(uint8_t *buffer, uint16_t size);
    int readFrameAfterWait(uint8_t *buffer,
                           uint16_t size,
                           uint32_t timeout_ms,
                           VL53L9CXFrameWaitResult *result = nullptr,
                           VL53L9CXFrameWaitMode mode = VL53L9CX_FRAME_WAIT_POLL,
                           uint16_t poll_interval_ms = 5U);
    int ackFrame();

    vl53l9_arduino_device_t *device();
    const vl53l9_arduino_platform_error_t *lastPlatformError() const;

    static const char *errorName(int status);
    static const char *frameWaitModeName(VL53L9CXFrameWaitMode mode);

    static constexpr uint16_t rawResolutionForBinning(uint8_t binning)
    {
        switch (binning)
        {
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
                return 0U;
        }
    }

    static constexpr uint8_t rawWidthForBinning(uint8_t binning)
    {
        switch (binning)
        {
            case 2:
                return 54U;
            case 4:
                return 24U;
            case 6:
                return 18U;
            case 8:
                return 12U;
            case 12:
                return 8U;
            case 24:
                return 4U;
            default:
                return 0U;
        }
    }

    static constexpr uint8_t outputWidthForBinning(uint8_t binning)
    {
        return rawWidthForBinning(binning);
    }

    static constexpr uint8_t outputHeightForBinning(uint8_t binning)
    {
        switch (binning)
        {
            case 2:
                return 42U;
            case 4:
                return 20U;
            case 6:
                return 14U;
            case 8:
                return 10U;
            case 12:
                return 6U;
            case 24:
                return 4U;
            default:
                return 0U;
        }
    }

    static constexpr uint8_t outputYOffsetForBinning(uint8_t binning)
    {
        switch (binning)
        {
            case 4:
                return 2U;
            case 12:
                return 1U;
            default:
                return 0U;
        }
    }

    static constexpr uint16_t rawBufferSizeForBinning(uint8_t binning)
    {
        const uint16_t resolution = rawResolutionForBinning(binning);
        return resolution == 0U
                   ? 0U
                   : static_cast<uint16_t>((resolution * 6U) + (resolution / 2U) + VL53L9_STATUS_SIZE);
    }

private:
    bool interruptIsActive(VL53L9CXFrameWaitMode mode, int *level) const;

    vl53l9_arduino_device_t device_;
    int8_t interrupt_pin_;
};

#endif // VL53L9CX_ARDUINO_H
