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
    bool interrupt_latched = false;
    uint32_t interrupt_count = 0U;
};

struct VL53L9CXRawFrameSummary
{
    uint16_t raw_size = 0U;
    uint16_t raw_resolution = 0U;
    uint16_t nonzero_depth_pixels = 0U;
    uint16_t flagged_pixels = 0U;
    uint16_t min_depth = 0U;
    uint16_t average_depth = 0U;
    uint16_t max_depth = 0U;
    uint16_t center_depth = 0U;
    uint16_t center_amplitude = 0U;
    uint16_t center_ambient = 0U;
    uint32_t frame_counter = 0U;
    uint16_t temperature_raw = 0U;
};

class VL53L9CXRawFrame;

class VL53L9CX
{
public:
    VL53L9CX();

    void setAddress(uint8_t dynamic_address);
    void setPowerConfig(vl53l9_vdda_t vdda, vl53l9_vddio_t vddio, uint32_t ext_clock_hz);
    void setInterruptPin(int8_t interrupt_pin);
    bool attachFrameInterrupt(VL53L9CXFrameWaitMode mode);
    void detachFrameInterrupt();
    void clearFrameInterruptLatch();
    bool frameInterruptAttached() const;
    bool frameInterruptLatched() const;
    uint32_t frameInterruptCount() const;

    int init();
    int getDeviceId(uint32_t *device_id);
    int getHwConfig(vl53l9_hw_config_t *config);
    int setHwConfig(const vl53l9_hw_config_t &config);

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
    static const char *syncModeName(vl53l9_sync_mode_t mode);
    static const char *powerModeName(vl53l9_power_mode_t mode);
    static const char *contextName(vl53l9_context_t context);
    static const char *frameWaitModeName(VL53L9CXFrameWaitMode mode);
    static const char *outputInterfaceName(bool output_interface);
    static const char *signalingModeName(bool signaling_mode);
    static const char *interruptPadModeName(bool interrupt_pad_mode);
    static void printHwConfig(Print &out, const vl53l9_hw_config_t &config);
    static void printPlatformError(Print &out, const vl53l9_arduino_platform_error_t *error);

    static uint16_t readLe16(const uint8_t *data);
    static uint32_t readLe32(const uint8_t *data);
    static uint16_t depthAtOutputPixel(const uint8_t *buffer, uint8_t binning, uint8_t x, uint8_t y);
    static bool summarizeRawFrame(const uint8_t *buffer,
                                  uint16_t size,
                                  uint8_t binning,
                                  VL53L9CXRawFrameSummary *summary);
    static bool printRawDepthGrid(Print &out,
                                  const uint8_t *buffer,
                                  uint8_t binning,
                                  uint16_t max_output_pixels = 64U);
    static bool printRawFrameSummary(Print &out,
                                     const uint8_t *buffer,
                                     uint16_t size,
                                     uint16_t frame_index,
                                     uint8_t binning);

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

    static constexpr uint8_t rawHeightForBinning(uint8_t binning)
    {
        const uint16_t resolution = rawResolutionForBinning(binning);
        const uint8_t width = rawWidthForBinning(binning);
        return (resolution == 0U || width == 0U) ? 0U : static_cast<uint8_t>(resolution / width);
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
    bool interrupt_attached_;
    VL53L9CXFrameWaitMode interrupt_wait_mode_;
};

class VL53L9CXRawFrame
{
public:
    VL53L9CXRawFrame();
    VL53L9CXRawFrame(const uint8_t *buffer, uint16_t size, uint8_t binning);

    void reset(const uint8_t *buffer, uint16_t size, uint8_t binning);
    bool valid() const;

    const uint8_t *data() const;
    uint16_t size() const;
    uint8_t binning() const;
    uint16_t rawResolution() const;
    uint8_t rawWidth() const;
    uint8_t rawHeight() const;
    uint8_t outputWidth() const;
    uint8_t outputHeight() const;

    uint16_t rawDepthAt(uint16_t raw_index) const;
    uint16_t depthAt(uint8_t x, uint8_t y) const;
    uint16_t amplitudeAt(uint8_t x, uint8_t y) const;
    uint16_t ambientAt(uint8_t x, uint8_t y) const;
    uint32_t frameCounter() const;
    uint16_t temperatureRaw() const;
    bool summary(VL53L9CXRawFrameSummary *summary) const;
    bool printSummary(Print &out, uint16_t frame_index) const;
    bool printDepthGrid(Print &out, uint16_t max_output_pixels = 64U) const;

private:
    uint16_t rawIndexForOutputPixel(uint8_t x, uint8_t y) const;

    const uint8_t *buffer_;
    uint16_t size_;
    uint8_t binning_;
};

#endif // VL53L9CX_ARDUINO_H
