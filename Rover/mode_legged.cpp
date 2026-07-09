#include "Rover.h"
#include "mode.h"
#include <AP_Biomimetic/AP_Biomimetic.h>

bool ModeLegged::_enter()
{
    AP_Biomimetic *bio = AP::biomimetic();
    if (bio == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "ModeLegged: AP_Biomimetic not available");
        return false;
    }
    _stand_done = false;
    return true;
}

void ModeLegged::update()
{
    // biomimetic control runs via dedicated 50Hz scheduler task
    // see Rover::biomimetic_update()
}

void ModeLegged::update_biomimetic()
{
    AP_Biomimetic *bio = AP::biomimetic();
    if (bio == nullptr) {
        return;
    }
    if (!_stand_done) {
        _stand_done = bio->stand();
    }
    bio->update();
}

void ModeLegged::_exit()
{
}
