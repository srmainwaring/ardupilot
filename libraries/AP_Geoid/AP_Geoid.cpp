#include "AP_Geoid.h"

#if AP_GEOID_AVAILABLE

#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include <AP_Geoid/Geoid.h>

extern const AP_HAL::HAL& hal;

AP_Geoid *AP_Geoid::singleton;

#if APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define GEOID_ENABLE_DEFAULT 0
#else
#define GEOID_ENABLE_DEFAULT 1
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Geoid::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Geoid data enable
    // @Description: enable geoid data. This enables the vehicle storing a database of geoid data on the SD card. The geoid data is requested from the ground station as needed, and stored for later use on the SD card. To be useful the ground station must support GEOID_REQUEST messages and have access to a geoid database.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Geoid, enable, GEOID_ENABLE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

// constructor
AP_Geoid::AP_Geoid()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (singleton != nullptr) {
        AP_HAL::panic("Geoid must be singleton");
    }
#endif
    singleton = this;
}

void AP_Geoid::init(void) {
    create_file_path();
    allocate();
}

bool AP_Geoid::geoid_height(const Location &loc, float &height)
{
    ftype lat = loc.lat * 1.0E-7;
    ftype lon = loc.lng * 1.0E-7;
    // ftype alt_m = loc.alt * 1.0E-2;
    ftype height_;
    bool status = (*geoid)(lat, lon, height_);
    height = static_cast<float>(height_);
    return status;
}

bool AP_Geoid::height_above_ellipsoid(const Location &loc, float height_geoid, float &height_ellipsoid)
{
  ftype lat = loc.lat * 1.0E-7;
  ftype lon = loc.lng * 1.0E-7;
  ftype h_out;
  bool status = geoid->ConvertHeight(lat, lon, height_geoid,
      GeographicLib::Geoid::GEOIDTOELLIPSOID, h_out);
  height_ellipsoid = static_cast<float>(h_out);
  return status;
}

bool AP_Geoid::height_above_geoid(const Location &loc, float height_ellipsoid, float &height_geoid)
{
  ftype lat = loc.lat * 1.0E-7;
  ftype lon = loc.lng * 1.0E-7;
  ftype h_out;
  bool status = geoid->ConvertHeight(lat, lon, height_ellipsoid,
      GeographicLib::Geoid::ELLIPSOIDTOGEOID, h_out);
  height_geoid = static_cast<float>(h_out);
  return status;
}

bool AP_Geoid::allocate(void)
{
    if (enable == 0 || memory_alloc_failed) {
        return false;
    }
    if (geoid != nullptr) {
        return true;
    }
    geoid = new GeographicLib::Geoid(file_path, true);
    if (geoid == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Geoid: Allocation failed");
        memory_alloc_failed = true;
        return false;
    }
    return true;
}

void AP_Geoid::create_file_path(void)
{
    if (file_path == nullptr) {
        const char* geoid_dir = hal.util->get_custom_geoid_directory();
        if (geoid_dir == nullptr) {
            geoid_dir = HAL_BOARD_GEOID_DIRECTORY;
        }
        if (asprintf(&file_path, "%s/egm96-5.pgm", geoid_dir) <= 0) {
            io_failure = true;
            file_path = nullptr;
            return;
        }
    }
    if (file_path == nullptr) {
        io_failure = true;
        return;
    }
    char *p = &file_path[strlen(file_path)-12];
    if (*p != '/') {
        io_failure = true;
        return;
    }

    // create directory if need be
    if (!directory_created) {
        *p = 0;
        directory_created = !AP::FS().mkdir(file_path);
        *p = '/';

        if (!directory_created) {
            if (errno == EEXIST) {
                // directory already existed
                directory_created = true;
            } else {
                // if we didn't succeed at making the directory, then IO failed
                io_failure = true;
                return;
            }
        }
    }
}


namespace AP {

AP_Geoid &geoid()
{
    return *AP_Geoid::get_singleton();
}

};

#endif // AP_GEOID_AVAILABLE
