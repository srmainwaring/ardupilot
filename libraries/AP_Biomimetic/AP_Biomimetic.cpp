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
    AP_GROUPINFO("KNEE_STAND",  2, AP_Biomimetic, p_knee_stand_deg,       20.0f),
    AP_GROUPINFO("ANK_P_STAND", 3, AP_Biomimetic, p_ank_pitch_stand_deg,  15.0f),
    AP_GROUPINFO("STAND_RATE",  4, AP_Biomimetic, p_stand_rate_dps,       20.0f),
    AP_GROUPINFO("BAL_EN",      5, AP_Biomimetic, p_balance_enable,           0),
    AP_GROUPINFO("GAIT_EN",     6, AP_Biomimetic, p_gait_enable,              0),
    AP_GROUPINFO("GAIT_PERIOD", 7, AP_Biomimetic, p_gait_period_s,          1.5f),
    AP_GROUPINFO("GAIT_LEN",    8, AP_Biomimetic, p_gait_step_len_deg,     15.0f),
    AP_GROUPINFO("GAIT_HGT",    9, AP_Biomimetic, p_gait_step_height_deg, 12.0f),
    AP_GROUPEND
};

AP_Biomimetic::AP_Biomimetic()
    : _initialized(false)
    , _standing(false)
    , _gait_phase(0.0f)
    , _lipm_e(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Biomimetic: multiple instances");
    }
    _singleton = this;
    memset(_state, 0, sizeof(_state));
    memset(_cmd,   0, sizeof(_cmd));
    memset(_stand_targets, 0, sizeof(_stand_targets));
    memset(_lipm_x, 0, sizeof(_lipm_x));
    _bal_integral = 0.0f;
    _bal_last_pitch = 0.0f;
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

// Map AP_Biomimetic internal joint index -> ArduPilotPlugin output channel
// (channel order is fixed by op3_with_ardupilot/model.sdf <control channel='N'>).
//
// AP_Biomimetic layout per side (0-5 / 6-11): hip_roll, hip_yaw, hip_pitch,
// knee, ank_pitch, ank_roll
// SDF channel layout per side (0-5 / 6-11):   hip_yaw, hip_roll, hip_pitch,
// knee, ank_pitch, ank_roll
//
// Only hip_roll/hip_yaw are swapped within each leg; the rest line up 1:1.
const uint8_t AP_Biomimetic::_joint_to_sdf_channel[AP_BIOMIMETIC_NUM_JOINTS] = {
    1, 0, 2, 3, 4, 5,      // left leg:  hip_roll->ch1, hip_yaw->ch0, rest unchanged
    7, 6, 8, 9, 10, 11     // right leg: hip_roll->ch7, hip_yaw->ch6, rest unchanged
};

void AP_Biomimetic::_write_servos()
{
    // Inverse of ArduPilotPlugin::UpdateMotorCommands() pwm->cmd conversion
    // (see ardupilot_gazebo-1/src/ArduPilotPlugin.cc):
    //   raw_cmd = (pwm - servo_min) / (servo_max - servo_min)   in [0,1]
    //   cmd_rad = multiplier * (raw_cmd + offset)
    // SDF values for all 12 leg <control> blocks (model.sdf / op3_direct.sdf):
    //   multiplier=3.14159, offset=-0.5, servo_min=1100, servo_max=1900
    // giving cmd_rad range = multiplier * [-0.5, +0.5] = [-1.5708, +1.5708] rad
    // (+/-90 deg), enough headroom for every BIOM joint limit (max +/-60deg,
    // knee 0-90deg).
    //
    // Inverting for deg -> pwm:
    //   cmd_rad  = deg * DEG_TO_RAD
    //   raw_cmd  = cmd_rad / multiplier - offset      (offset = -0.5, so this adds 0.5)
    //   pwm      = servo_min + raw_cmd * (servo_max - servo_min)
    const float multiplier  = 1.571f;
    const float offset      = -0.5f;
    const float servo_min   = 1000.0f;
    const float servo_max   = 2000.0f;

    for (uint8_t i = 0; i < AP_BIOMIMETIC_NUM_JOINTS; i++) {
        const uint8_t chan = _joint_to_sdf_channel[i];
        const float cmd_rad = _cmd[i].target_deg * DEG_TO_RAD;
        const float raw_cmd = cmd_rad / multiplier - offset;
        float pwm_f = servo_min + raw_cmd * (servo_max - servo_min);
        uint16_t pwm = (uint16_t)constrain_float(pwm_f, servo_min, servo_max);
        SRV_Channels::set_output_pwm_chan(chan, pwm);
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
    // TEMP DEBUG -- remove after diagnosing hip_pitch stuck-at-0 issue
    static uint32_t _debug_last_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _debug_last_ms > 1000) {
        _debug_last_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
            "BIOM dbg: tgt2=%.1f cmd2=%.1f tgt3=%.1f cmd3=%.1f",
            (double)_stand_targets[2], (double)_cmd[2].target_deg,
            (double)_stand_targets[3], (double)_cmd[3].target_deg);
    }
    if (all_close && !_standing) {
        _standing = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_Biomimetic: standing");
    }
    return all_close;
}

void AP_Biomimetic::balance_update()
{
    // LIPM Preview Controller -- DARE-precomputed gains
    // Ported from humanoid-ardupilot-sitl/scripts/preview_control.py
    //
    // LIPM state: x = [com_pos, com_vel, com_acc]
    // ZMP output: zmp = com_pos - (zc/g) * com_acc
    //
    // Discrete-time system (dt=0.02, zc=0.38m, g=9.81):
    //   A = [[1, 0.02, 0], [0, 1, 0.02], [0, 0, 1]]
    //   B = [0, 0, 0.02]
    //   C = [1, 0, -zc/g]
    //
    // Gains from DARE solve (Qe=1.0, R=1e-6):
    //   Ke = 478.505159
    //   Kx = [10739.791188, 2517.886340, 84.613451]
    //   Gp[20] = preview gains (see below)
    //
    // Balance PID from Python prototype (balance.lua / zmp_gait_controller.py):
    //   Kp=0.10, Ki=0.001, Kd=0.01, clamp +/-0.25 rad

    if (p_balance_enable != 1) {
        return;
    }

    // DARE-precomputed preview gains Gp[20]
    static const float Gp[AP_BIOMIMETIC_PREVIEW_N] = {
        0.771033f,       // [0]
        -46571.691562f,  // [1]
        -33850.834160f,  // [2]
        -10572.121153f,  // [3]
        5378.544754f,    // [4]
        12069.694302f,   // [5]
        12826.731213f,   // [6]
        11082.482319f,   // [7]
        8888.735678f,    // [8]
        7063.506599f,    // [9]
        5737.273046f,    // [10]
        4787.992946f,    // [11]
        4067.107296f,    // [12]
        3474.461269f,    // [13]
        2960.131740f,    // [14]
        2504.922563f,    // [15]
        2102.780943f,    // [16]
        1750.885074f,    // [17]
        1445.889509f,    // [18]
        1183.393053f,    // [19]
    };

    // LIPM state update -- x = [com_pos, com_vel, com_acc]
    // A matrix (dt=0.02)
    const float dt   = 1.0f / AP_BIOMIMETIC_UPDATE_HZ;
    const float zc_g = 0.038745f;  // zc/g = 0.38/9.81

    // ZMP reference is zero for static balance (stand in place)
    // When gait runs this will be fed from the footstep planner
    float zmp_ref[AP_BIOMIMETIC_PREVIEW_N] = {};  // all zeros = stand in place

    // Current ZMP from LIPM state
    float zmp_now = _lipm_x[0] - zc_g * _lipm_x[2];

    // Integral of ZMP error
    _lipm_e += zmp_now - zmp_ref[0];

    // Preview sum
    float preview_sum = 0.0f;
    for (uint8_t i = 0; i < AP_BIOMIMETIC_PREVIEW_N; i++) {
        preview_sum += Gp[i] * zmp_ref[i];
    }

    // Control input
    const float Ke      = 478.505159f;
    const float Kx0     = 10739.791188f;
    const float Kx1     = 2517.886340f;
    const float Kx2     = 84.613451f;

    float u = -Ke * _lipm_e
              - Kx0 * _lipm_x[0]
              - Kx1 * _lipm_x[1]
              - Kx2 * _lipm_x[2]
              - preview_sum;

    // Propagate LIPM state
    // x_new = A*x + B*u
    float x_new[3];
    x_new[0] = _lipm_x[0] + dt * _lipm_x[1];
    x_new[1] = _lipm_x[1] + dt * _lipm_x[2];
    x_new[2] = _lipm_x[2] + dt * u;
    _lipm_x[0] = x_new[0];
    _lipm_x[1] = x_new[1];
    _lipm_x[2] = x_new[2];

    // Read pitch and roll from AHRS for PID balance correction
    const AP_AHRS &ahrs = AP::ahrs();
    float pitch_rad = ahrs.get_pitch();

    // PID balance from Python prototype: Kp=0.10, Ki=0.001, Kd=0.01
    const float Kp_bal = 0.10f;
    const float Ki_bal = 0.001f;
    const float Kd_bal = 0.01f;

    _bal_integral = constrain_float(_bal_integral + pitch_rad * dt, -0.3f, 0.3f);
    float bal_d   = (pitch_rad - _bal_last_pitch) / dt;
    _bal_last_pitch = pitch_rad;

    float bal = constrain_float(
        Kp_bal * pitch_rad + Ki_bal * _bal_integral + Kd_bal * bal_d,
        -0.25f, 0.25f);

    // CoM lateral position from LIPM drives hip roll
    // hr = com_y * 2.0 from zmp_gait_controller.py
    // For static balance com_y stays near zero -- this is the lateral stub
    float com_y    = 0.0f;  // TODO: add lateral LIPM when gait runs
    float hr_left  = -com_y * 2.0f + (-bal * 0.3f);
    float hr_right =  com_y * 2.0f + ( bal * 0.3f);

    // Ankle pitch correction from LIPM com_x
    float ankle_correction_deg = constrain_float(
        _lipm_x[0] * RAD_TO_DEG, -10.0f, 10.0f);

    // joint layout per side: hip_roll=0 hip_yaw=1 hip_pitch=2 knee=3 ank_pitch=4 ank_roll=5
    // left:  hip_roll=0, ank_pitch=4
    // right: hip_roll=6, ank_pitch=10
    set_joint_cmd_deg(0,  _stand_targets[0]  + hr_left  * RAD_TO_DEG);
    set_joint_cmd_deg(6,  _stand_targets[6]  + hr_right * RAD_TO_DEG);
    set_joint_cmd_deg(4,  _stand_targets[4]  + ankle_correction_deg);
    set_joint_cmd_deg(10, _stand_targets[10] + ankle_correction_deg);
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
            ank_pitch_deg = hip_pitch_deg * 0.5f;
        } else {
            // swing half of cycle: foot lifts, swings forward, plants
            float t = (leg_phase - 0.5f) / 0.5f;  // 0..1 across swing half
            // knee bend peaks at mid-swing, zero at start/end of swing
            knee_deg      = step_hgt_deg * sinf(t * float(M_PI));
            hip_pitch_deg = step_len_deg * (t - 0.5f);
            ank_pitch_deg = -knee_deg * 0.3f;  // small compensation to keep foot level
        }

        // lateral weight shift: lean toward stance leg BEFORE swing phase
        float hip_roll_deg = 0.0f;
        // weight shift driven by global phase so stance leg gets weight BEFORE swing lifts
        float shift_t = sinf(leg_phase * 2.0f * float(M_PI));
        hip_roll_deg = (side == 0 ? 1.0f : -1.0f) * 12.0f * shift_t;
        set_joint_cmd_deg(base + 0, _stand_targets[base + 0] + hip_roll_deg);
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
