#include "Copter.h"

#if MODE_PLANNED_RTL_ENABLED

bool ModePlannedRTL::init(bool ignore_checks)
{
    return mode_loiter.init(ignore_checks);
}

#if AC_PRECLAND_ENABLED
void ModePlannedRTL::set_precision_loiter_enabled(bool value)
{
    return mode_loiter.set_precision_loiter_enabled(value);
}

bool ModePlannedRTL::do_precision_loiter()
{
    return mode_loiter.do_precision_loiter();
}

void ModePlannedRTL::precision_loiter_xy()
{
    mode_loiter.precision_loiter_xy();
}
#endif  // AC_PRECLAND_ENABLED

void ModePlannedRTL::run()
{
    mode_loiter.run();
}

float ModePlannedRTL::wp_distance_m() const
{
    return mode_loiter.wp_distance_m();
}

float ModePlannedRTL::wp_bearing_deg() const
{
    return mode_loiter.wp_bearing_deg();
}

#endif  // MODE_PLANNED_RTL_ENABLED
