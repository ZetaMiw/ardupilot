/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for ST VL53L3CX lidar

  Based on StampFly implementation and ST VL53LX API
 */
#include "AP_RangeFinder_VL53L3CX.h"

#if AP_RANGEFINDER_VL53L3CX_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

static const uint8_t MEASUREMENT_TIME_MS = 50; // Start continuous readings at a rate of one measurement every 50 ms

// VL53L3CX I2C addresses and register definitions
#define VL53LX_SOFT_RESET                                  0x0000
#define VL53LX_I2C_SLAVE__DEVICE_ADDRESS                   0x0001
#define VL53LX_IDENTIFICATION__MODEL_ID                    0x010F
#define VL53LX_IDENTIFICATION__MODULE_TYPE                 0x0110
#define VL53LX_FIRMWARE__SYSTEM_STATUS                     0x00E5
#define VL53LX_SYSTEM__MODE_START                          0x0087
#define VL53LX_RESULT__RANGE_STATUS                        0x0089
#define VL53LX_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0  0x0096
#define VL53LX_SYSTEM__INTERRUPT_CLEAR                     0x0086
#define VL53LX_GPIO__TIO_HV_STATUS                         0x0031
#define VL53LX_PAD_I2C_HV__EXTSUP_CONFIG                   0x002E
#define VL53LX_RANGE_CONFIG__VCSEL_PERIOD_A                0x0060
#define VL53LX_RANGE_CONFIG__VCSEL_PERIOD_B                0x0063
#define VL53LX_OSC_MEASURED__FAST_OSC__FREQUENCY           0x0006
#define VL53LX_RESULT__OSC_CALIBRATE_VAL                   0x00DE
#define VL53LX_SYSTEM__INTERMEASUREMENT_PERIOD             0x006C
#define VL53LX_PHASECAL_CONFIG__TIMEOUT_MACROP             0x004B
#define VL53LX_RANGE_CONFIG__TIMEOUT_MACROP_A              0x005E
#define VL53LX_RANGE_CONFIG__TIMEOUT_MACROP_B              0x0061
#define VL53LX_MM_CONFIG__TIMEOUT_MACROP_A                 0x004A
#define VL53LX_MM_CONFIG__TIMEOUT_MACROP_B                 0x004D
#define VL53LX_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND       0x0008
#define VL53LX_VHV_CONFIG__INIT                            0x000B
#define VL53LX_PHASECAL_CONFIG__OVERRIDE                   0x004D
#define VL53LX_PHASECAL_RESULT__VCSEL_START                0x00D8
#define VL53LX_CAL_CONFIG__VCSEL_START                     0x0047

AP_RangeFinder_VL53L3CX::AP_RangeFinder_VL53L3CX(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev))
    , sum_mm(0)
    , counter(0)
    , calibrated(false)
    , fast_osc_frequency(0)
    , osc_calibrate_val(0)
{}

/*
   detect if a VL53L3CX rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_VL53L3CX::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, DistanceMode mode)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_VL53L3CX *sensor
        = NEW_NOTHROW AP_RangeFinder_VL53L3CX(_state, _params, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->take_blocking();

    if (!sensor->check_id() || !sensor->init(mode)) {
        sensor->dev->get_semaphore()->give();
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->give();

    return sensor;
}

// check sensor ID registers
bool AP_RangeFinder_VL53L3CX::check_id(void)
{
    uint8_t v1, v2;
    if (!(read_register(VL53LX_IDENTIFICATION__MODEL_ID, v1) &&
          read_register(VL53LX_IDENTIFICATION__MODULE_TYPE, v2))) {
        return false;
    }

    // VL53L3CX has Model ID = 0xEB (different from VL53L1X's 0xEA)
    if ((v1 != 0xEB && v1 != 0xEA) ||  // Accept both for compatibility
        (v2 != 0xAA && v2 != 0xCC)) {   // Accept different module types
        return false;
    }
    printf("Detected VL53L3CX on bus 0x%x (Model ID: 0x%02X, Module Type: 0x%02X)\n",
           unsigned(dev->get_bus_id()), v1, v2);
    return true;
}

bool AP_RangeFinder_VL53L3CX::reset(void) {
    if (dev->get_bus_id() != 0x29) {
        // if sensor is on a different port than the default do not reset sensor otherwise we will lose the address.
        // we assume it is already configured.
        return true;
    }
    if (!write_register(VL53LX_SOFT_RESET, 0x00)) {
        return false;
    }
    hal.scheduler->delay_microseconds(100);
    if (!write_register(VL53LX_SOFT_RESET, 0x01)) {
        return false;
    }
    hal.scheduler->delay(1000);
    return true;
}

bool AP_RangeFinder_VL53L3CX::VL53LX_WaitDeviceBooted(void)
{
    uint8_t status = 0;
    uint16_t timeout = 0;

    do {
        if (!read_register(VL53LX_FIRMWARE__SYSTEM_STATUS, status)) {
            return false;
        }
        hal.scheduler->delay(1);
        timeout++;
    } while ((status & 0x01) == 0 && timeout < 1000);

    return timeout < 1000;
}

bool AP_RangeFinder_VL53L3CX::VL53LX_DataInit(void)
{
    // Basic initialization - setup for I2C 2.8V operation
    uint8_t pad_i2c_hv_extsup_config = 0;
    if (read_register(VL53LX_PAD_I2C_HV__EXTSUP_CONFIG, pad_i2c_hv_extsup_config)) {
        if (!write_register(VL53LX_PAD_I2C_HV__EXTSUP_CONFIG, pad_i2c_hv_extsup_config | 0x01)) {
            return false;
        }
    }

    // Store oscillator info for later use
    if (!read_register16(VL53LX_OSC_MEASURED__FAST_OSC__FREQUENCY, fast_osc_frequency)) {
        return false;
    }
    if (!read_register16(VL53LX_RESULT__OSC_CALIBRATE_VAL, osc_calibrate_val)) {
        return false;
    }

    return true;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_VL53L3CX::init(DistanceMode mode)
{
    // we need to do resets and delays in order to configure the sensor, don't do this if we are trying to fast boot
    if (hal.util->was_watchdog_armed()) {
        return false;
    }

    if (!reset()) {
        return false;
    }

    if (!VL53LX_WaitDeviceBooted()) {
        return false;
    }

    if (!VL53LX_DataInit()) {
        return false;
    }

    if (!VL53LX_SetDistanceMode(mode)) {
        return false;
    }

    if (!VL53LX_SetMeasurementTimingBudgetMicroSeconds(33000)) {
        return false;
    }

    if (!VL53LX_StartMeasurement()) {
        return false;
    }

    // call timer() every MEASUREMENT_TIME_MS. We expect new data to be available every MEASUREMENT_TIME_MS
    dev->register_periodic_callback(MEASUREMENT_TIME_MS * 1000,
                                    FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L3CX::timer, void));

    return true;
}

// set distance mode to Short, Medium, or Long
bool AP_RangeFinder_VL53L3CX::VL53LX_SetDistanceMode(DistanceMode distance_mode)
{
    // VL53L3CX distance mode configuration
    // These values are derived from ST's VL53LX API

    switch (distance_mode) {
      case DistanceMode::Short:
            // Short range mode configuration
            if (!(write_register(VL53LX_RANGE_CONFIG__VCSEL_PERIOD_A, 0x07) &&
                  write_register(VL53LX_RANGE_CONFIG__VCSEL_PERIOD_B, 0x05))) {
                return false;
            }
            break;

        case DistanceMode::Medium:
            // Medium range mode configuration (StampFly uses this)
            if (!(write_register(VL53LX_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B) &&
                  write_register(VL53LX_RANGE_CONFIG__VCSEL_PERIOD_B, 0x09))) {
                return false;
            }
            break;

        case DistanceMode::Long:
            // Long range mode configuration
            if (!(write_register(VL53LX_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F) &&
                  write_register(VL53LX_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D))) {
                return false;
            }
            break;

        default:
            return false;
    }

    return true;
}

bool AP_RangeFinder_VL53L3CX::VL53LX_SetMeasurementTimingBudgetMicroSeconds(uint32_t budget_us)
{
    // Simplified timing budget setting
    // For VL53L3CX, the timing budget affects measurement accuracy and speed
    // 33ms is a good balance as used by StampFly

    if (budget_us < 20000 || budget_us > 1000000) {
        return false;
    }

    // The actual register programming would go here
    // For now, we accept the value
    return true;
}

bool AP_RangeFinder_VL53L3CX::VL53LX_StartMeasurement(void)
{
    // Clear any pending interrupts
    if (!write_register(VL53LX_SYSTEM__INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    // Start continuous ranging mode
    if (!write_register(VL53LX_SYSTEM__MODE_START, 0x40)) {
        return false;
    }

    return true;
}

bool AP_RangeFinder_VL53L3CX::VL53LX_ClearInterruptAndStartMeasurement(void)
{
    return write_register(VL53LX_SYSTEM__INTERRUPT_CLEAR, 0x01);
}

bool AP_RangeFinder_VL53L3CX::VL53LX_GetMeasurementDataReady(uint8_t &data_ready)
{
    uint8_t gpio_status = 0;
    if (!read_register(VL53LX_GPIO__TIO_HV_STATUS, gpio_status)) {
        return false;
    }

    data_ready = (gpio_status & 0x01) == 0 ? 1 : 0;
    return true;
}

bool AP_RangeFinder_VL53L3CX::VL53LX_GetRangingMeasurementData(uint16_t &range_mm)
{
    uint8_t range_status = 0;

    if (!read_register(VL53LX_RESULT__RANGE_STATUS, range_status)) {
        return false;
    }

    if (!read_register16(VL53LX_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, range_mm)) {
        return false;
    }

    // Apply correction gain (same as VL53L1X)
    range_mm = ((uint32_t)range_mm * 2011 + 0x0400) / 0x0800;

    // Check range status
    uint8_t status = range_status & 0x1F;
    if (status != 9) { // 9 = Range Valid
        return false;
    }

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_VL53L3CX::get_reading(uint16_t &reading_mm)
{
    uint8_t data_ready = 0;
    uint8_t tries = 10;

    while (!dataReady()) {
        tries--;
        hal.scheduler->delay(1);
        if (tries == 0) {
            return false;
        }
    }

    if (!VL53LX_GetRangingMeasurementData(reading_mm)) {
        return false;
    }

    if (!VL53LX_ClearInterruptAndStartMeasurement()) {
        return false;
    }

    return true;
}

bool AP_RangeFinder_VL53L3CX::dataReady(void)
{
    uint8_t data_ready = 0;
    return VL53LX_GetMeasurementDataReady(data_ready) && data_ready;
}

bool AP_RangeFinder_VL53L3CX::setupManualCalibration(void)
{
    // Placeholder for manual calibration if needed
    return true;
}

bool AP_RangeFinder_VL53L3CX::read_register(uint16_t reg, uint8_t &value)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    return dev->transfer(b, 2, &value, 1);
}

bool AP_RangeFinder_VL53L3CX::read_register16(uint16_t reg, uint16_t &value)
{
    uint16_t v = 0;
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    if (!dev->transfer(b, 2, (uint8_t *)&v, 2)) {
        return false;
    }
    value = be16toh(v);
    return true;
}

bool AP_RangeFinder_VL53L3CX::read_register32(uint16_t reg, uint32_t &value)
{
    uint32_t v = 0;
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    if (!dev->transfer(b, 2, (uint8_t *)&v, 4)) {
        return false;
    }
    value = be32toh(v);
    return true;
}

bool AP_RangeFinder_VL53L3CX::write_register(uint16_t reg, uint8_t value)
{
    uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
    return dev->transfer(b, 3, nullptr, 0);
}

bool AP_RangeFinder_VL53L3CX::write_register16(uint16_t reg, uint16_t value)
{
    uint8_t b[4] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), uint8_t(value >> 8), uint8_t(value & 0xFF) };
    return dev->transfer(b, 4, nullptr, 0);
}

bool AP_RangeFinder_VL53L3CX::write_register32(uint16_t reg, uint32_t value)
{
    uint8_t b[6] = { uint8_t(reg >> 8),
                     uint8_t(reg & 0xFF),
                     uint8_t((value >> 24) & 0xFF),
                     uint8_t((value >> 16) & 0xFF),
                     uint8_t((value >>  8) & 0xFF),
                     uint8_t((value)       & 0xFF) };
    return dev->transfer(b, 6, nullptr, 0);
}

bool AP_RangeFinder_VL53L3CX::read_multi(uint16_t reg, uint8_t *data, uint8_t len)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    return dev->transfer(b, 2, data, len);
}

bool AP_RangeFinder_VL53L3CX::write_multi(uint16_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    // Would need to concatenate register address and data
    // For now, not implemented
    return false;
}

/*
  timer called at 20Hz
*/
void AP_RangeFinder_VL53L3CX::timer(void)
{
    uint16_t range_mm;
    if ((get_reading(range_mm)) && (range_mm <= 6000)) { // VL53L3CX max range is 6m
        WITH_SEMAPHORE(_sem);
        sum_mm += range_mm;
        counter++;
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_VL53L3CX::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (counter > 0) {
        state.distance_m = (sum_mm * 0.001f) / counter;
        state.last_reading_ms = AP_HAL::millis();
        update_status();
        sum_mm = 0;
        counter = 0;
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        // if no updates for 0.2s set no-data
        set_status(RangeFinder::Status::NoData);
    }
}

#endif  // AP_RANGEFINDER_VL53L3CX_ENABLED
