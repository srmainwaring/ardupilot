-- gait.lua
-- Step sequencer for ArduHumanoid
-- Phases: STAND -> SHIFT_R -> STEP_L -> STAND -> SHIFT_L -> STEP_R

local gait = {}

-- Gait parameters
gait.HIP_STAND   = -0.09
gait.KNEE_STAND  =  0.35
gait.KNEE_LIFT   =  0.38
gait.HIP_STEP    =  0.05
gait.DURATION    =  3.0  -- seconds per phase

-- Phases
gait.PHASE = {
    STAND   = 1,
    SHIFT_R = 2,
    STEP_L  = 3,
    SHIFT_L = 4,
    STEP_R  = 5,
}

-- State
local phase = gait.PHASE.STAND
local phase_start = 0.0

function gait.reset()
    phase = gait.PHASE.STAND
    phase_start = millis() / 1000.0
end

-- Returns joint targets table for current phase
function gait.update()
    local now = millis() / 1000.0
    local elapsed = now - phase_start

    if elapsed >= gait.DURATION then
        -- Advance to next phase
        phase = (phase % 5) + 1
        phase_start = now
    end

    local targets = {
        l_hip_pitch = gait.HIP_STAND,
        r_hip_pitch = gait.HIP_STAND,
        l_knee      = gait.KNEE_STAND,
        r_knee      = gait.KNEE_STAND,
    }

    if phase == gait.PHASE.SHIFT_R then
        targets.r_hip_pitch = gait.HIP_STAND + gait.HIP_STEP

    elseif phase == gait.PHASE.STEP_L then
        targets.r_hip_pitch = gait.HIP_STAND + gait.HIP_STEP
        targets.l_knee      = gait.KNEE_LIFT
        targets.l_hip_pitch = gait.HIP_STAND - gait.HIP_STEP

    elseif phase == gait.PHASE.SHIFT_L then
        targets.l_hip_pitch = gait.HIP_STAND + gait.HIP_STEP

    elseif phase == gait.PHASE.STEP_R then
        targets.l_hip_pitch = gait.HIP_STAND + gait.HIP_STEP
        targets.r_knee      = gait.KNEE_LIFT
        targets.r_hip_pitch = gait.HIP_STAND - gait.HIP_STEP
    end

    return targets
end

return gait
