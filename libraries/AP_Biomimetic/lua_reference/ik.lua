-- ik.lua
-- 2D inverse kinematics for ArduHumanoid
-- Solves hip and knee angles for a given foot position
-- Leg geometry: L1 = L2 = 0.20m (thigh = shin)

local ik = {}

ik.L1 = 0.20  -- thigh length (m)
ik.L2 = 0.20  -- shin length (m)

-- Solve IK for one leg
-- x = forward offset, z = vertical (negative = down)
-- returns hip_pitch, knee angles in radians, or nil if unreachable
function ik.solve(x, z)
    local L1, L2 = ik.L1, ik.L2
    local d = math.sqrt(x*x + z*z)

    -- Clamp to reachable range
    d = math.max(math.abs(L1 - L2) + 0.001,
                 math.min(L1 + L2 - 0.001, d))

    -- Knee angle (law of cosines)
    local cos_knee = (L1*L1 + L2*L2 - d*d) / (2*L1*L2)
    cos_knee = math.max(-1.0, math.min(1.0, cos_knee))
    local knee = math.pi - math.acos(cos_knee)

    -- Hip angle
    local alpha = math.atan(x, -z)
    local cos_alpha2 = (L1*L1 + d*d - L2*L2) / (2*L1*d)
    cos_alpha2 = math.max(-1.0, math.min(1.0, cos_alpha2))
    local alpha2 = math.acos(cos_alpha2)
    local hip = alpha - alpha2

    return hip, knee
end

return ik
