#pragma once

#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>
#include <AP_HAL/Semaphores.h>

class AP_SurfaceDistance {
public:
    AP_SurfaceDistance(Rotation rot, uint8_t i) :
        instance(i),
        rotation(rot)
    {};

    void update();

    // check if the last healthy range finder reading is too old to be considered valid
    bool data_stale(void);

    // helper to check that rangefinder was last reported as enabled and healthy
    bool enabled_and_healthy(void) const;

    // Check that there is a rangefinder in the correct orientation configured, used for pre-arm checks
    // when the rangefinder may not be reporting healthy (e.g. on ground reporting out of range low)
    bool rangefinder_configured(void) const;

    // get inertially interpolated rangefinder height
    bool get_rangefinder_height_interpolated_m(float& height_m, const uint32_t oor_low_timeout_ms = 0);

    bool enabled;                           // not to be confused with rangefinder enabled, this state is to be set by the vehicle.
    bool alt_healthy;                       // true if we can trust the altitude from the rangefinder
    float alt_m;                            // tilt compensated altitude (in cm) from rangefinder
    float inertial_alt_m;                   // inertial alt at time of last rangefinder sample
    LowPassFilterFloat alt_m_filt {0.5};    // altitude filter
    float alt_glitch_protected_m;           // last glitch protected altitude
    int8_t glitch_count;                    // non-zero number indicates rangefinder is glitching
    uint32_t glitch_cleared_ms;             // system time glitch cleared
    float terrain_offset_m;                 // filtered terrain offset (e.g. terrain's height above EKF origin)

private:
#if HAL_LOGGING_ENABLED
    void Log_Write(void) const;
#endif

    // multi-thread access support
    HAL_Semaphore sem;

    bool rangefinder_is_config;
    bool has_been_healthy;
    uint32_t out_of_range_low_ms;           // keep track of the rangefinder state. When using the inertially interpolated rangefinder reading sometimes it is acceptable to use the rangefinder when it is reporting low
    const uint8_t instance;
    uint8_t status;
    uint32_t last_healthy_ms;

    const Rotation rotation;
};
