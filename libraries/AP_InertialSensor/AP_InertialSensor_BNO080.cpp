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

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_BNO080.h"

extern const AP_HAL::HAL& hal;

#define ACCEL_BACKEND_SAMPLE_RATE   1000
#define GYRO_BACKEND_SAMPLE_RATE    1000

const uint32_t ACCEL_BACKEND_PERIOD_US = 1000000UL / ACCEL_BACKEND_SAMPLE_RATE;
const uint32_t GYRO_BACKEND_PERIOD_US = 1000000UL / GYRO_BACKEND_SAMPLE_RATE;

const uint32_t ACCEL_BACKEND_PERIOD_MS = 1000UL / ACCEL_BACKEND_SAMPLE_RATE;
const uint32_t GYRO_BACKEND_PERIOD_MS = 1000UL / GYRO_BACKEND_SAMPLE_RATE;


extern const AP_HAL::HAL& hal;

AP_InertialSensor_BNO080::AP_InertialSensor_BNO080(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                   enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
{
    hal.console->printf("BNO080: construct\n");
}

AP_InertialSensor_Backend *
AP_InertialSensor_BNO080::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation)
{
    hal.console->printf("BNO080: probe device\n");

    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_BNO080(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_BNO080::start()
{
    hal.console->printf("BNO080: start\n");

    bool ret = false;

    _dev->get_semaphore()->take_blocking();

    ret = _configure_accel();
    if (!ret) {
        AP_HAL::panic("BNO080: Unable to configure accelerometer");
    }

    ret = _configure_gyro();
    if (!ret) {
        AP_HAL::panic("BNO080: Unable to configure gyro");
    }

    ret = _configure_fifo();
    if (!ret) {
        AP_HAL::panic("BNO080: Unable to configure FIFO");
    }

    _dev->get_semaphore()->give();

    // register with front end
    if (!_imu.register_accel(_accel_instance, ACCEL_BACKEND_SAMPLE_RATE,
            _dev->get_bus_id_devtype(DEVTYPE_INS_BNO080)) ||
        !_imu.register_gyro(_gyro_instance, GYRO_BACKEND_SAMPLE_RATE,
            _dev->get_bus_id_devtype(DEVTYPE_INS_BNO080))) {
        return;
    }

    // setup callbacks
    _periodic_handle = _dev->register_periodic_callback(
        ACCEL_BACKEND_PERIOD_US,
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BNO080::_poll_data, void));
}

bool AP_InertialSensor_BNO080::update()
{
    hal.console->printf("BNO080: update\n");
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

bool AP_InertialSensor_BNO080::_hardware_init()
{
    hal.console->printf("BNO080: hardware init\n");

    // device settings
    hal.console->printf("BNO080: bus_num: 0x%02hhx\n", _dev->bus_num());
    hal.console->printf("BNO080: address: 0x%02hhx\n", _dev->get_bus_address());

    bool ret = false;
    _dev->get_semaphore()->take_blocking();

    uint16_t time_between_reports_ms = 10;
    
    ret = _bno080.begin(_dev.get());
    if (ret) {
        _bno080.enableAccelerometer(time_between_reports_ms);
        _bno080.enableGyro(time_between_reports_ms);
        _bno080.enableMagnetometer(time_between_reports_ms);
        _bno080.enableRawAccelerometer(time_between_reports_ms);
        _bno080.enableRawGyro(time_between_reports_ms);
        _bno080.enableRawMagnetometer(time_between_reports_ms);
    }

    _dev->get_semaphore()->give();

    return ret;
}

bool AP_InertialSensor_BNO080::_init()
{
    hal.console->printf("BNO080: init\n");

    bool ret = _hardware_init();
    if (!ret) {
        hal.console->printf("BNO080: failed to init\n");
    }

    return ret;
}

bool AP_InertialSensor_BNO080::_configure_accel()
{
    hal.console->printf("BNO080: configure_accel\n");

    // setup sensor orientation from probe()
    set_accel_orientation(_accel_instance, _rotation);

    //! @todo(srmainwaring) get scale
    _accel_scale = 1.0;

    return true;
}

bool AP_InertialSensor_BNO080::_configure_gyro()
{
    hal.console->printf("BNO080: configure_gyro\n");

    // setup sensor orientation from probe()
    set_gyro_orientation(_gyro_instance, _rotation);

    //! @todo(srmainwaring) get scale
    _gyro_scale = 1.0;

    return true;
}

bool AP_InertialSensor_BNO080::_configure_fifo()
{
    hal.console->printf("BNO080: configure_gyro\n");

    return true;
}

void AP_InertialSensor_BNO080::_poll_data(void)
{
  _read_fifo();
}

void AP_InertialSensor_BNO080::_read_fifo(void)
{
    if (_bno080.dataAvailable()) {
        uint16_t gx = _bno080.getRawGyroX();
        uint16_t gy = _bno080.getRawGyroY();
        uint16_t gz = _bno080.getRawGyroZ();
        hal.console->printf("BNO080: gx: %d gy: %d gz: %d\n", gx, gy, gz);
    }
}

