#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Servo_Telem/AP_Servo_Telem.h>
#include <SRV_Channel/SRV_Channel.h>

#define AP_BIOMIMETIC_NUM_JOINTS    12
#define AP_BIOMIMETIC_UPDATE_HZ     50

class AP_Biomimetic {
public:
    AP_Biomimetic();
    CLASS_NO_COPY(AP_Biomimetic);
    static AP_Biomimetic *get_singleton() { return _singleton; }

    void update();
    void set_joint_cmd_deg(uint8_t joint_idx, float deg);
    bool get_joint_state(uint8_t joint_idx, float &pos_deg, float &vel_dps, float &torque_nm) const;
    bool stand();
    void balance_update();

    static const struct AP_Param::GroupInfo var_info[];
    AP_Float p_hip_pitch_stand_deg;
    AP_Float p_knee_stand_deg;
    AP_Float p_ank_pitch_stand_deg;
    AP_Float p_stand_rate_dps;
    AP_Int8  p_balance_enable;

private:
    struct JointState {
        float pos_deg;
        float vel_dps;
        float torque_nm;
        bool  valid;
    };
    struct JointCmd {
        float target_deg;
    };

    JointState _state[AP_BIOMIMETIC_NUM_JOINTS];
    JointCmd   _cmd[AP_BIOMIMETIC_NUM_JOINTS];
    float      _stand_targets[AP_BIOMIMETIC_NUM_JOINTS];

    static const float _joint_min_deg[AP_BIOMIMETIC_NUM_JOINTS];
    static const float _joint_max_deg[AP_BIOMIMETIC_NUM_JOINTS];

    void _read_telem();
    void _write_servos();
    void _update_stand_targets();

    bool _initialized;
    bool _standing;

    static AP_Biomimetic *_singleton;
};

namespace AP {
    AP_Biomimetic *biomimetic();
}
