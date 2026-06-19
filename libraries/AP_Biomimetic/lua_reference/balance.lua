-- balance.lua
-- Pitch balance controller for ArduHumanoid
-- Reads ATTITUDE from EKF, adjusts hip pitch to correct lean

local balance = {}

-- PID gains (tuned from Python gait_controller.py)
balance.Kp = 0.08
balance.Ki = 0.001
balance.Kd = 0.01

-- State
local integral = 0.0
local last_error = 0.0
local last_time = 0.0

-- Reset controller state
function balance.reset()
    integral = 0.0
    last_error = 0.0
    last_time = millis() / 1000.0
end

-- Update balance, returns hip pitch correction in radians
function balance.update(target_pitch)
    local now = millis() / 1000.0
    local dt = now - last_time
    if dt <= 0 or dt > 0.5 then
        last_time = now
        return 0.0
    end

    local ahrs_pitch = ahrs:get_pitch()
    if not ahrs_pitch then return 0.0 end

    local error = target_pitch - ahrs_pitch
    integral = integral + error * dt
    local derivative = (error - last_error) / dt

    last_error = error
    last_time = now

    local correction = balance.Kp * error
                     + balance.Ki * integral
                     + balance.Kd * derivative

    -- Clamp correction
    return math.max(-0.2, math.min(0.2, correction))
end

return balance
