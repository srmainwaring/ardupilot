#include "mode.h"
#include "Plane.h"

bool ModePlannedRTL::_enter()
{
    return mode_loiter.enter();
}

void ModePlannedRTL::update()
{
    mode_loiter.update();
}

bool ModePlannedRTL::isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc)
{
    return mode_loiter.isHeadingLinedUp(loiterCenterLoc, targetLoc);
}

bool ModePlannedRTL::isHeadingLinedUp_cd(const int32_t bearing_cd)
{
    return mode_loiter.isHeadingLinedUp_cd(bearing_cd);
}

bool ModePlannedRTL::isHeadingLinedUp_cd(const int32_t bearing_cd, const int32_t heading_cd)
{
    return mode_loiter.isHeadingLinedUp_cd(bearing_cd, heading_cd);
}

void ModePlannedRTL::navigate()
{
    mode_loiter.navigate();
}

void ModePlannedRTL::update_target_altitude()
{
    mode_loiter.update_target_altitude();
}
