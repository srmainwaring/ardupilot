#include "AP_Biomimetic.h"
#include <AP_Servo_Telem/AP_Servo_Telem.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

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
    AP_GROUPINFO("GAIT_EN",     6, AP_Biomimetic, p_gait_enable,              0),
    AP_GROUPINFO("GAIT_PERIOD", 7, AP_Biomimetic, p_gait_period_s,          1.5f),
    AP_GROUPINFO("GAIT_LEN",    8, AP_Biomimetic, p_gait_step_len_deg,     15.0f),
    AP_GROUPINFO("GAIT_HGT",    9, AP_Biomimetic, p_gait_step_height_deg, 20.0f),
    AP_GROUPEND
};

AP_Biomimetic::AP_Biomimetic()
    : _initialized(false)
    , _standing(false)
    , _gait_phase(0.0f)
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
    if (_standing && p_gait_enable == 1) {
        gait_step();
    } else if (_standing && p_balance_enable == 1) {
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
    // ZMP/LIPM balance stub -- July milestone
    //
    // What this does right now:
    //   1. reads CoM tilt estimate from AHRS (pitch = forward lean)
    //   2. computes a proportional ankle pitch correction
    //   3. applies it symmetrically to both ankle joints (index 4 and 10)
    //
    // What slots in here in July:
    //   - DARE-precomputed LIPM gain replaces the proportional gain
    //   - lateral (roll) axis correction added alongside pitch
    //   - ZMP stays inside support polygon check added before correction

    if (p_balance_enable != 1) {
        return;
    }

    // read pitch angle from AHRS -- positive means leaning forward
    const AP_AHRS &ahrs = AP::ahrs();
    float pitch_rad = ahrs.get_pitch();

    // proportional gain: 1 deg of lean -> 1 deg of ankle correction
    // TODO July: replace with DARE-precomputed LIPM gain
    const float kp = 1.0f;

    float ankle_correction_deg = kp * RAD_TO_DEG * pitch_rad;

    // clamp correction to +/- 10 deg so it cannot fight the stand targets
    ankle_correction_deg = constrain_float(ankle_correction_deg, -10.0f, 10.0f);

    // joint layout per side: hip_roll=0 hip_yaw=1 hip_pitch=2 knee=3 ank_pitch=4 ank_roll=5
    // left ankle = index 4, right ankle = index 10
    const uint8_t left_ankle  = 4;
    const uint8_t right_ankle = 10;

    set_joint_cmd_deg(left_ankle,
        _stand_targets[left_ankle]  + ankle_correction_deg);
    set_joint_cmd_deg(right_ankle,
        _stand_targets[right_ankle] + ankle_correction_deg);
}

void AP_Biomimetic::gait_step()
{
    // Static gait primitive -- Phase 1 scope per DESIGN.md
    // Fixed step sequence, no reactive ZMP correction.
    //
    // Single phase variable 0.0-1.0 loops continuously.
    // Left leg uses phase directly, right leg is offset by 0.5 (half cycle)
    // so the two legs alternate stance/swing -- this is what produces walking
    // rather than hopping.
    //
    // Per-leg phase is split into 4 quarters:
    //   0.00-0.25  Stance -> Lift   (knee bends, hip starts swinging back to front)
    //   0.25-0.50  Lift   -> Swing  (knee at max bend, hip sweeps forward)
    //   0.50-0.75  Swing  -> Plant  (knee starts straightening, hip continues forward)
    //   0.75-1.00  Plant  -> Stance (foot back on ground, hip sweeps back for next push)

    if (p_gait_enable != 1) {
        return;
    }

    const float dt = 1.0f / AP_BIOMIMETIC_UPDATE_HZ;
    const float period = MAX(p_gait_period_s.get(), 0.1f);

    _gait_phase += dt / period;
    if (_gait_phase >= 1.0f) {
        _gait_phase -= 1.0f;
    }

    const float step_len_deg = p_gait_step_len_deg.get();
    const float step_hgt_deg = p_gait_step_height_deg.get();

    // joint layout per side: hip_roll=0 hip_yaw=1 hip_pitch=2 knee=3 ank_pitch=4 ank_roll=5
    for (uint8_t side = 0; side < 2; side++) {
        const uint8_t base = side * 6;
        // right leg (side 1) is offset by half a cycle from left leg (side 0)
        float leg_phase = _gait_phase + (side == 1 ? 0.5f : 0.0f);
        if (leg_phase >= 1.0f) {
            leg_phase -= 1.0f;
        }

        float hip_pitch_deg;
        float knee_deg;
        float ank_pitch_deg;

        if (leg_phase < 0.5f) {
            // stance half of cycle: foot on ground, hip sweeps back to front
            // as the body moves forward over the planted foot
            float t = leg_phase / 0.5f;  // 0..1 across stance half
            hip_pitch_deg = step_len_deg * (0.5f - t);
            knee_deg      = 0.0f;
            ank_pitch_deg = 0.0f;
        } else {
            // swing half of cycle: foot lifts, swings forward, plants
            float t = (leg_phase - 0.5f) / 0.5f;  // 0..1 across swing half
            // knee bend peaks at mid-swing, zero at start/end of swing
            knee_deg      = step_hgt_deg * sinf(t * float(M_PI));
            hip_pitch_deg = step_len_deg * (t - 0.5f);
            ank_pitch_deg = -knee_deg * 0.3f;  // small compensation to keep foot level
        }

        set_joint_cmd_deg(base + 2, _stand_targets[base + 2] + hip_pitch_deg);
        set_joint_cmd_deg(base + 3, _stand_targets[base + 3] + knee_deg);
        set_joint_cmd_deg(base + 4, _stand_targets[base + 4] + ank_pitch_deg);
    }
}

namespace AP {
AP_Biomimetic *biomimetic()
{
    return AP_Biomimetic::get_singleton();
}
}
