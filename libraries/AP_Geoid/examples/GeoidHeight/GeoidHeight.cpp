/*
  example for AP_Geoid

  requires the geoid data file egm96-5.pgm in directory ./geoids

  if GeographicLib is installed this may be found at:
  /usr/local/share/GeographicLib/geoids
*/

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Common/Location.h>
#include <AP_Geoid/AP_Geoid.h>
#include <AP_Geoid/Geoid.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_Dummy.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  dummy vehicle
*/
class DummyVehicle : public AP_Vehicle {
public:
    void init();
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override;
    uint8_t get_mode() const override;
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &task_count, uint32_t &log_bit) override;
    void init_ardupilot() override;
    void load_parameters() override;
    const AP_Int32 &get_log_bitmask() override;
    const struct LogStructure *get_log_structures() const override;
    uint8_t get_num_log_structures() const override;

private:
    AP_Int32 unused_log_bitmask;
    static const struct LogStructure log_structure[];
};

const struct LogStructure DummyVehicle::log_structure[] = {
    LOG_COMMON_STRUCTURES
};

void DummyVehicle::init() {
    BoardConfig.init();
    ins.init(100);
    ahrs.init();
    logger.init(get_log_bitmask(), get_log_structures(), get_num_log_structures());
}

bool DummyVehicle::set_mode(const uint8_t new_mode, const ModeReason reason) {
    return true;
}

uint8_t DummyVehicle::get_mode() const {
    return 1;
}

void DummyVehicle::get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &task_count, uint32_t &log_bit) {
}

void DummyVehicle::init_ardupilot() {
}

void DummyVehicle::load_parameters() {
}

const AP_Int32 &DummyVehicle::get_log_bitmask() {
    return unused_log_bitmask;
}

const struct LogStructure *DummyVehicle::get_log_structures() const {
    return log_structure;
}

uint8_t DummyVehicle::get_num_log_structures() const {
    return ARRAY_SIZE(log_structure);
}

/*
  initialise systems
*/
static AP_AHRS ahrs{AP_AHRS::FLAG_ALWAYS_USE_EKF};
static AP_Geoid geoid;
static AP_SerialManager serial_manager;
static DummyVehicle vehicle;

/*
  sketch setup and loop
*/
void setup()
{
    hal.console->printf("Geoid Height\n");
    hal.scheduler->delay(100);

    vehicle.init();
    serial_manager.init();
    geoid.init();
}

void loop()
{
    static uint32_t last_t_us;
    static uint32_t last_geoid_us;
    uint32_t now = AP_HAL::micros();

    if (last_t_us == 0) {
        last_t_us = now;
        return;
    }
    last_t_us = now;

    // update geoid at 1Hz
    if (now - last_geoid_us > 1000 * 1000UL) {
        float lat = 42.0;
        float lng = -75.0;
        float height_above_geoid = 20.0;
        Location loc(lat * 1E7, lng * 1E7, height_above_geoid * 1E2,
            Location::AltFrame::ABSOLUTE);

        // convert height above geoid to height above ellipsoid
        float geoid_height;
        float height_above_ellipsoid;
        if (geoid.geoid_height(loc, geoid_height)) {
            height_above_ellipsoid = (height_above_geoid +
                  GeographicLib::Geoid::GEOIDTOELLIPSOID * geoid_height);
            hal.console->printf("height_above_ellipsoid: %.6f\n", height_above_ellipsoid);
        }

        // convert height above geoid to height above ellipsoid directly
        if (geoid.height_above_ellipsoid(loc, height_above_geoid, height_above_ellipsoid)) {
            hal.console->printf("height_above_ellipsoid: %.6f\n", height_above_ellipsoid);
        }

        // convert height above ellipsoid to height above geoid directly
        if (geoid.height_above_geoid(loc, height_above_ellipsoid, height_above_geoid)) {
            hal.console->printf("height_above_geoid: %.6f\n", height_above_geoid);
        }

        last_geoid_us = now;
    }

    ahrs.update();
}

/*
  parameters, dummy gcs, and main entry point
*/
const struct AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
static GCS_Dummy _gcs;

AP_HAL_MAIN();
