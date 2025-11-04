#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L3CX_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_VL53L3CX : public AP_RangeFinder_Backend
{

public:
    enum class DistanceMode { Short, Medium, Long, Unknown };

    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev, DistanceMode mode);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // VL53L3CX specific device handle structure
    struct VL53LX_Dev_t {
        uint8_t i2c_slave_address;
        uint16_t comms_speed_khz;
    };

    AP_RangeFinder_VL53L3CX(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev);

    // check sensor ID
    bool check_id(void);

    // reset sensor
    bool reset(void);

    // initialise sensor
    bool init(DistanceMode mode);

    // set distance mode
    bool setDistanceMode(DistanceMode distance_mode);

    // set measurement timing budget
    bool setMeasurementTimingBudget(uint32_t budget_us);

    // get measurement timing budget
    bool getMeasurementTimingBudget(uint32_t &budget);

    // start continuous ranging
    bool startContinuous(uint32_t period_ms);

    // check if data is ready
    bool dataReady(void);

    // get reading
    bool get_reading(uint16_t &reading_mm);

    // setup manual calibration
    bool setupManualCalibration(void);

    // read/write helpers for VL53L3CX API compatibility
    bool VL53LX_WaitDeviceBooted(void);
    bool VL53LX_DataInit(void);
    bool VL53LX_SetDistanceMode(DistanceMode mode);
    bool VL53LX_SetMeasurementTimingBudgetMicroSeconds(uint32_t budget_us);
    bool VL53LX_StartMeasurement(void);
    bool VL53LX_GetMeasurementDataReady(uint8_t &data_ready);
    bool VL53LX_GetRangingMeasurementData(uint16_t &range_mm);
    bool VL53LX_ClearInterruptAndStartMeasurement(void);

    // register read/write functions
    bool read_register(uint16_t reg, uint8_t &value);
    bool read_register16(uint16_t reg, uint16_t &value);
    bool read_register32(uint16_t reg, uint32_t &value);
    bool write_register(uint16_t reg, uint8_t value);
    bool write_register16(uint16_t reg, uint16_t value);
    bool write_register32(uint16_t reg, uint32_t value);

    // multi-byte read
    bool read_multi(uint16_t reg, uint8_t *data, uint8_t len);
    bool write_multi(uint16_t reg, const uint8_t *data, uint8_t len);

    // timer callback
    void timer(void);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    uint16_t sum_mm;
    uint16_t counter;
    bool calibrated;

    HAL_Semaphore _sem;

    // VL53L3CX specific parameters
    uint16_t fast_osc_frequency;
    uint16_t osc_calibrate_val;
};

#endif  // AP_RANGEFINDER_VL53L3CX_ENABLED
