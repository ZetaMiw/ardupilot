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
 * AP_OpticalFlow_PMW3901.h - PixArt PMW3901MB-TXQT optical flow sensor support
 * Based on Bitcraze PMW3901 driver
 */

#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_PMW3901_ENABLED

#include "AP_OpticalFlow_Backend.h"

class AP_OpticalFlow_PMW3901 : public AP_OpticalFlow_Backend
{
public:
    // constructor
    AP_OpticalFlow_PMW3901(AP_OpticalFlow &_frontend, AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev);

    // initialise the sensor
    void init() override;

    // read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // detect if the sensor is available
    static AP_OpticalFlow_Backend *detect(AP_OpticalFlow &_frontend);

private:
    // PMW3901 Registers
    enum regAddr : uint8_t {
        PRODUCT_ID       = 0x00,
        REVISION_ID      = 0x01,
        MOTION           = 0x02,
        DELTA_X_L        = 0x03,
        DELTA_X_H        = 0x04,
        DELTA_Y_L        = 0x05,
        DELTA_Y_H        = 0x06,
        SQUAL            = 0x07,
        RAW_DATA_SUM     = 0x08,
        MAXIMUM_RAW_DATA = 0x09,
        MINIMUM_RAW_DATA = 0x0A,
        SHUTTER_LOWER    = 0x0B,
        SHUTTER_UPPER    = 0x0C,
        OBSERVATION      = 0x15,
        MOTION_BURST     = 0x16,
        POWER_UP_RESET   = 0x3A,
        SHUTDOWN         = 0x3B,
        RAW_DATA_GRAB    = 0x58,
        RAW_DATA_GRAB_STATUS = 0x59,
        INVERSE_PRODUCT_ID   = 0x5F
    };

    // Motion burst data structure
    struct MotionBurst {
        uint8_t motion;
        uint8_t observation;
        int16_t delta_x;
        int16_t delta_y;
        uint8_t squal;
        uint8_t raw_data_sum;
        uint8_t max_raw_data;
        uint8_t min_raw_data;
        uint16_t shutter;
    } __attribute__((packed));

    // register read/write
    uint8_t read_register(uint8_t reg);
    void write_register(uint8_t reg, uint8_t value);

    // perform initialization sequence
    bool perform_init_sequence();

    // read motion burst data
    bool read_motion_burst(MotionBurst &burst);

    // timer to read sensor
    void timer();

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    // accumulated motion data
    struct {
        int32_t x;
        int32_t y;
        uint32_t last_update_ms;
    } _accum;

    // motion scale factor (converts sensor units to pixels)
    // PMW3901 reports motion in counts, needs to be scaled to pixels
    // Field of view is 42 degrees
    static constexpr float SCALER = 1.0f / 5.0f;  // Empirical scaling factor
};

#endif  // AP_OPTICALFLOW_PMW3901_ENABLED
