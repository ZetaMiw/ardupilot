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
 * AP_OpticalFlow_PMW3901.cpp - PixArt PMW3901MB-TXQT optical flow sensor support
 * Based on Bitcraze PMW3901 driver and datasheet
 */

#include "AP_OpticalFlow_PMW3901.h"

#if AP_OPTICALFLOW_PMW3901_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <utility>

extern const AP_HAL::HAL& hal;

// Product ID for PMW3901
#define PMW3901_PRODUCT_ID    0x49
#define PMW3901_REVISION_ID   0x00

// constructor
AP_OpticalFlow_PMW3901::AP_OpticalFlow_PMW3901(AP_OpticalFlow &_frontend, AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev) :
    AP_OpticalFlow_Backend(_frontend),
    _dev(std::move(dev))
{
    _accum.x = 0;
    _accum.y = 0;
    _accum.last_update_ms = 0;
}

// detect the device
AP_OpticalFlow_Backend *AP_OpticalFlow_PMW3901::detect(AP_OpticalFlow &_frontend)
{
    // look for the device on SPI bus
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev = hal.spi->get_device("pmw3901");
    if (!dev) {
        return nullptr;
    }

    AP_OpticalFlow_PMW3901 *sensor = NEW_NOTHROW AP_OpticalFlow_PMW3901(_frontend, std::move(dev));
    if (sensor == nullptr) {
        return nullptr;
    }

    sensor->init();

    if (!sensor->_dev) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

// initialise the sensor
void AP_OpticalFlow_PMW3901::init()
{
    if (!_dev) {
        return;
    }

    // get the semaphore
    WITH_SEMAPHORE(_dev->get_semaphore());

    // power up reset
    write_register(POWER_UP_RESET, 0x5A);
    hal.scheduler->delay(50);

    // read product ID
    uint8_t product_id = read_register(PRODUCT_ID);
    uint8_t revision_id = read_register(REVISION_ID);
    uint8_t inverse_product_id = read_register(INVERSE_PRODUCT_ID);

    if (product_id != PMW3901_PRODUCT_ID || inverse_product_id != (~PMW3901_PRODUCT_ID & 0xFF)) {
        _dev = nullptr;
        return;
    }

    hal.console->printf("PMW3901: Detected (Product ID: 0x%02X, Revision: 0x%02X)\n",
                       product_id, revision_id);

    // perform initialization sequence
    if (!perform_init_sequence()) {
        _dev = nullptr;
        return;
    }

    // register timer callback at 100Hz (10ms)
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_PMW3901::timer, void));
}

// read a register
uint8_t AP_OpticalFlow_PMW3901::read_register(uint8_t reg)
{
    uint8_t tx[2] = { reg, 0x00 };
    uint8_t rx[2] = { 0, 0 };

    _dev->transfer(tx, 2, rx, 2);
    hal.scheduler->delay_microseconds(50);  // tSRAD: Read address to data output delay

    return rx[1];
}

// write a register
void AP_OpticalFlow_PMW3901::write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), value };  // Set MSB for write

    _dev->transfer(tx, 2, nullptr, 0);
    hal.scheduler->delay_microseconds(50);  // tSWW/tSWR: Write to write/read delay
}

// perform initialization sequence from datasheet
bool AP_OpticalFlow_PMW3901::perform_init_sequence()
{
    // Initialization sequence from PMW3901 datasheet
    // This configures the sensor for optimal performance

    write_register(0x7F, 0x00);
    write_register(0x61, 0xAD);
    write_register(0x7F, 0x03);
    write_register(0x40, 0x00);
    write_register(0x7F, 0x05);
    write_register(0x41, 0xB3);
    write_register(0x43, 0xF1);
    write_register(0x45, 0x14);
    write_register(0x5B, 0x32);
    write_register(0x5F, 0x34);
    write_register(0x7B, 0x08);
    write_register(0x7F, 0x06);
    write_register(0x44, 0x1B);
    write_register(0x40, 0xBF);
    write_register(0x4E, 0x3F);
    write_register(0x7F, 0x08);
    write_register(0x65, 0x20);
    write_register(0x6A, 0x18);
    write_register(0x7F, 0x09);
    write_register(0x4F, 0xAF);
    write_register(0x5F, 0x40);
    write_register(0x48, 0x80);
    write_register(0x49, 0x80);
    write_register(0x57, 0x77);
    write_register(0x60, 0x78);
    write_register(0x61, 0x78);
    write_register(0x62, 0x08);
    write_register(0x63, 0x50);
    write_register(0x7F, 0x0A);
    write_register(0x45, 0x60);
    write_register(0x7F, 0x00);
    write_register(0x4D, 0x11);
    write_register(0x55, 0x80);
    write_register(0x74, 0x1F);
    write_register(0x75, 0x1F);
    write_register(0x4A, 0x78);
    write_register(0x4B, 0x78);
    write_register(0x44, 0x08);
    write_register(0x45, 0x50);
    write_register(0x64, 0xFF);
    write_register(0x65, 0x1F);
    write_register(0x7F, 0x14);
    write_register(0x65, 0x67);
    write_register(0x66, 0x08);
    write_register(0x63, 0x70);
    write_register(0x7F, 0x15);
    write_register(0x48, 0x48);
    write_register(0x7F, 0x07);
    write_register(0x41, 0x0D);
    write_register(0x43, 0x14);
    write_register(0x4B, 0x0E);
    write_register(0x45, 0x0F);
    write_register(0x44, 0x42);
    write_register(0x4C, 0x80);
    write_register(0x7F, 0x10);
    write_register(0x5B, 0x02);
    write_register(0x7F, 0x07);
    write_register(0x40, 0x41);
    write_register(0x70, 0x00);

    hal.scheduler->delay(10);

    // Read motion register to clear it
    read_register(MOTION);

    return true;
}

// read motion burst data
bool AP_OpticalFlow_PMW3901::read_motion_burst(MotionBurst &burst)
{
    uint8_t tx[13] = { MOTION_BURST, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t rx[13];

    if (!_dev->transfer(tx, sizeof(tx), rx, sizeof(rx))) {
        return false;
    }

    hal.scheduler->delay_microseconds(50);

    burst.motion = rx[1];
    burst.observation = rx[2];
    burst.delta_x = (int16_t)((rx[4] << 8) | rx[3]);
    burst.delta_y = (int16_t)((rx[6] << 8) | rx[5]);
    burst.squal = rx[7];
    burst.raw_data_sum = rx[8];
    burst.max_raw_data = rx[9];
    burst.min_raw_data = rx[10];
    burst.shutter = (rx[12] << 8) | rx[11];

    return true;
}

// timer callback to read sensor
void AP_OpticalFlow_PMW3901::timer()
{
    MotionBurst burst;

    if (!read_motion_burst(burst)) {
        return;
    }

    // check if motion is valid
    if ((burst.motion & 0x80) == 0) {
        // no motion detected
        return;
    }

    // accumulate motion
    _accum.x += burst.delta_x;
    _accum.y += burst.delta_y;
    _accum.last_update_ms = AP_HAL::millis();
}

// update - read latest values from sensor
void AP_OpticalFlow_PMW3901::update()
{
    uint32_t now = AP_HAL::millis();

    // check for timeout
    if (now - _accum.last_update_ms > 200) {
        // no updates for 200ms
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    struct AP_OpticalFlow::OpticalFlow_state state {};

    // populate state structure
    state.surface_quality = 255;  // PMW3901 doesn't provide quality in same format

    // convert accumulated motion to flow rates
    // PMW3901 reports motion in counts, convert to rad/s
    // The sensor has a 42-degree field of view
    // Scale factor converts sensor counts to angular motion
    float dt = (now - _accum.last_update_ms) * 0.001f;
    if (dt > 0.001f) {
        // flowRate is in radians/sec
        state.flowRate.x = (_accum.x * SCALER) / dt;
        state.flowRate.y = (_accum.y * SCALER) / dt;
    }

    // bodyRate comes from the frontend
    state.bodyRate = AP::ahrs().get_gyro();

    _update_frontend(state);

    // reset accumulator
    _accum.x = 0;
    _accum.y = 0;
}

#endif  // AP_OPTICALFLOW_PMW3901_ENABLED
