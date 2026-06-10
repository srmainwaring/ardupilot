#include "AP_Biomimetic.h"
#include <AP_Servo_Telem/AP_Servo_Telem.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_Biomimetic *AP_Biomimetic::_singleton;

const float AP_Biomimetic::_joint_min_deg[AP_BIOMIMETIC_NUM_JOINTS] = {
    -45.0f, -45.0f, -60.0f,   0.0f, -45.0f, -45.0f,
    -45.0f, -45.0f, -60.0f,   0.0f, -45.0f, -45.0f
};
const float AP_Biomimetic::_joint_max_deg[AP_BIOMIMETIC_NUM_JOINTS] = {
     45.0f,  45.0f,  60.0f,  90.0f,  45.0f,  45.0f,
     45.0f,  45.0f,  60.0f,  90.0f,  45.0f,  45.0f
};

const AP_Param::GroupInfo AP_Biomimetic::var_info[] = {
    AP_GROUPINFO("HIP_P_STAND", 1, AP_Biomimetic, p_hip_pitch_stand_deg, -15.0f),
    AP_GROUPINFO("KNEE_STAND",  2, AP_Biomimetic, p_knee_stand_deg,       30.0f),
    AP_GROUPINFO("ANK_P_STAND", 3, AP_Biomimetic, p_ank_pitch_stand_deg,  15.0f),
    AP_GROUPINFO("STAND_RATE",  4, AP_Biomimetic, p_stand_rate_dps,       20.0f),
    AP_GROUPINFO("BAL_EN",      5, AP_Biomimetic, p_balance_enable,           0),
    AP_GROUPEND
};

AP_Biomimetic::AP_Biomimetic()
    : _initialized(false)
    , _standing(false)
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Biomimetic: multiple instances");
    }
    _singleton = this;
    memset(_state, 0, sizeof(_state));
    memset(_cmd,   0, sizeof(_cmd));
    memset(_stand_targets, 0, sizeof(_stand_targets));
}

void AP_Biomimetic::update()
{
    if (!_initialized) {
        _update_stand_targets();
        _initialized = true;
    }
    _read_telem();
    if (_standing && p_balance_enable == 1) {
        balance_update();
    }
    _write_servos();
}

void AP_Biomimetic::_read_telem()
{
    AP_Servo_Telem *telem = AP_Servo_Telem::get_singleton();
    if (telem == nullptr) {
        return;
    }
    for (uint8_t i = 0; i < AP_BIOMIMETIC_NUM_JOINTS; i++) {
        AP_Servo_Telem::TelemetryData td;
        if (telem->get_telem(i, td)) {
            _state[i].pos_deg   = td.measured_position;
            _state[i].vel_dps   = td.speed;
            _state[i].torque_nm = td.force;
            _state[i].valid     = true;
        }
    }
}

void AP_Biomimetic::_write_servos()
{
    for (uint8_t i = 0; i < AP_BIOMIMETIC_NUM_JOINTS; i++) {
        SRV_Channels::set_output_scaled(
            SRV_Channel::Function(SRV_Channel::k_none + i),
            _cmd[i].target_deg);
    }
}

void AP_Biomimetic::set_joint_cmd_deg(uint8_t joint_idx, float deg)
{
    if (joint_idx >= AP_BIOMIMETIC_NUM_JOINTS) {
        return;
    }
    _cmd[joint_idx].target_deg = constrain_float(deg,
        _joint_min_deg[joint_idx], _joint_max_deg[joint_idx]);
}

bool AP_Biomimetic::get_joint_state(uint8_t joint_idx, float &pos_deg, float &vel_dps, float &torque_nm) const
{
    if (joint_idx >= AP_BIOMIMETIC_NUM_JOINTS || !_state[joint_idx].valid) {
        return false;
    }
    pos_deg   = _state[joint_idx].pos_deg;
    vel_dps   = _state[joint_idx].vel_dps;
    torque_nm = _state[joint_idx].torque_nm;
    return true;
}

void AP_Biomimetic::_update_stand_targets()
{
    for (uint8_t side = 0; side < 2; side++) {
        uint8_t base = side * 6;
        _stand_targets[base + 0] = 0.0f;
        _stand_targets[base + 1] = 0.0f;
        _stand_targets[base + 2] = p_hip_pitch_stand_deg.get();
        _stand_targets[base + 3] = p_knee_stand_deg.get();
        _stand_targets[base + 4] = p_ank_pitch_stand_deg.get();
        _stand_targets[base + 5] = 0.0f;
    }
}

bool AP_Biomimetic::stand()
{
    _update_stand_targets();
    const float dt        = 1.0f / AP_BIOMIMETIC_UPDATE_HZ;
    const float max_step  = p_stand_rate_dps.get() * dt;
    bool        all_close = true;
    for (uint8_t i = 0; i < AP_BIOMIMETIC_NUM_JOINTS; i++) {
        float error = _stand_targets[i] - _cmd[i].target_deg;
        float step  = constrain_float(error, -max_step, max_step);
        set_joint_cmd_deg(i, _cmd[i].target_deg + step);
        if (fabsf(error) > 1.0f) {
            all_close = false;
        }
    }
    if (all_close && !_standing) {
        _standing = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_Biomimetic: standing");
    }
    return all_close;
}

void AP_Biomimetic::balance_update()
{
    // TODO July: LIPM/ZMP CoM correction via ankle pitch adjustment
}

namespace AP {
AP_Biomimetic *biomimetic()
{
    return AP_Biomimetic::get_singleton();
}
}
