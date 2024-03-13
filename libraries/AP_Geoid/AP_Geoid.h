#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

#ifndef AP_GEOID_AVAILABLE
#define AP_GEOID_AVAILABLE AP_FILESYSTEM_FILE_READING_ENABLED
#endif

#if AP_GEOID_AVAILABLE

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Logger/AP_Logger_config.h>

namespace GeographicLib {
    class Geoid;
} //  GeographicLib

class AP_Geoid {
public:
    AP_Geoid();

    // Do not allow copies
    CLASS_NO_COPY(AP_Geoid);

    static AP_Geoid *get_singleton(void) { return singleton; }

    // enum GeoidStatus {
    //     GeoidStatusDisabled  = 0, // not enabled
    //     GeoidStatusUnhealthy = 1, // no geoid data for current location
    //     GeoidStatusOK        = 2  // geoid data available
    // };

    static const struct AP_Param::GroupInfo var_info[];

    bool enabled() const { return enable; }
    void set_enabled(bool _enable) { enable.set(_enable); }

    // return status enum for health reporting
    // enum GeoidStatus status(void) const { return system_status; }

    void init(void);

    // returns true if initialisation failed because out-of-memory
    bool init_failed() const { return memory_alloc_failed; }

    bool geoid_height(const Location &loc, float &height);

    bool height_above_ellipsoid(const Location &loc, float height_geoid, float &height_ellipsoid);

    bool height_above_geoid(const Location &loc, float height_ellipsoid, float &height_geoid);

private:
    // allocate the geoid subsystem data
    bool allocate(void);

    // create the directory and file path
    void create_file_path(void);

    // parameters
    AP_Int8  enable;
    bool memory_alloc_failed;

    // do we have an IO failure
    volatile bool io_failure;

    // have we created the geoid directory?
    bool directory_created;

    char *file_path = nullptr;

    // geoid object
    GeographicLib::Geoid *geoid = nullptr;

    // status
    // enum GeoidStatus system_status = GeoidStatusDisabled;

    static AP_Geoid *singleton;
};

namespace AP {
    AP_Geoid &geoid();
};

#endif // AP_GEOID_AVAILABLE
