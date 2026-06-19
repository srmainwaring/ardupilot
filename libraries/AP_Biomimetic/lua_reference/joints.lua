-- joints.lua
-- Joint mapping and PWM helpers for ArduHumanoid
-- Channels match humanoid.param servo assignments

local joints = {}

-- Joint channel mapping
joints.CHANNEL = {
    l_hip_pitch  = 1,
    r_hip_pitch  = 2,
    l_knee       = 3,
    r_knee       = 4,
}

-- Standing pose (radians)
joints.STAND = {
    l_hip_pitch  = -0.09,
    r_hip_pitch  = -0.09,
    l_knee       =  0.35,
    r_knee       =  0.35,
}

-- Convert radians to PWM (1000-2000us)
-- multiplier=1.571, offset=-0.5 from SDF
function joints.rad_to_pwm(rad)
    local normalized = rad / 1.571 + 0.5
    return math.floor(1000 + normalized * 1000)
end

-- Set a joint by name to a position in radians
function joints.set(name, rad)
    local ch = joints.CHANNEL[name]
    if ch then
        SRV_Channels:set_output_pwm_chan_timeout(ch - 1, joints.rad_to_pwm(rad), 2000)
    end
end

-- Set all joints to standing pose
function joints.stand()
    for name, rad in pairs(joints.STAND) do
        joints.set(name, rad)
    end
end

return joints
