/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include "AP_InertialSensor_BNO080_Driver.h"

class AP_InertialSensor_BNO080 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;

    bool update() override;

private:
    AP_InertialSensor_BNO080(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                             enum Rotation rotation);

    /**
     * Try to perform initialization of the BNO080 device.
     *
     * The device semaphore must be taken and released by the caller.
     *
     * @return true on success, false otherwise.
     */
    bool _hardware_init();

    /**
     * Try to initialize this driver.
     *
     * Do sensor and other required initializations.
     *
     * @return true on success, false otherwise.
     */
    bool _init();

    /**
     * Configure accelerometer sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_accel();

    /**
     * Configure gyroscope sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_gyro();

    /**
     * Configure FIFO.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_fifo();

    /**
     * Device periodic callback to read data from the sensors.
     */
    void _poll_data();

    /**
     * Read samples from fifo
     */
    void _read_fifo();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    enum Rotation _rotation;
    AP_HAL::Device::PeriodicHandle _periodic_handle;

    BNO080 _bno080;

    uint8_t _accel_instance;
    float _accel_scale;

    uint8_t _gyro_instance;
    float _gyro_scale;
};
