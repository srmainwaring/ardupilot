# AP_Biomimetic Design Document
GSoC 2026 -- Neeta Misericordia
Updated: 2026-06-10

---

## What this project is

Prove ArduPilot can command a humanoid robot. Primary target is the ROBOTIS OP3 in
Gazebo Harmonic. Final two weeks port to Unitree H1. Same library, two robots, only
the config file changes.

---

## The problem Rhys flagged

ArduPilot sends servo commands and gets nothing back. Humanoid joints are different --
they report position, velocity, and torque every tick. A balance controller needs that
data to decide what to command next. There is no general method in AP for this today.
Building that return path is what this project adds.

---

## How the interface works

Commands go down, joint state comes back up:

    AP_Biomimetic  -->  SRV_Channels  -->  Gazebo JPC
                                                |
                                        JointStatePublisher
                                                |
                                       joint_state_bridge.py
                                                |
                                         DroneCAN actuator.Status
                                                |
    AP_Servo_Telem  <--  DroneCAN backend  <---+
                                                |
    AP_Biomimetic reads pos / vel / torque <---+

The IMU path (Gazebo to AP via ArduPilotPlugin) already exists.
This adds the joint state feedback path alongside it.

---

## AP_Servo_Telem -- already exists, no new library needed

Confirmed by reading libraries/AP_Servo_Telem/AP_Servo_Telem.h.
Has all fields needed:

- measured_position (degrees) -- joint position
- speed (degrees/sec) -- joint velocity
- force (Nm) -- joint torque
- Already logs to DataFlash via write_log() at 10 Hz
- Already accessible via get_telem() per servo index

The DroneCAN actuator.Status message fields map directly:

    actuator_id  ->  servo/joint index
    position     ->  measured_position
    speed        ->  speed
    force        ->  force

Wire actuator.Status into AP_Servo_Telem.update_telem_data() per joint.
AP_Biomimetic reads joint state via AP_Servo_Telem.get_telem().

---

## Why DroneCAN over extending SIM_JSON

SIM_JSON could work -- adding joint state is just new keytable rows. But it runs at
1000 Hz and ties the feedback rate to the physics loop. DroneCAN runs at a
configurable rate (50 Hz default), decoupled from physics, and matches real hardware
exactly. Rhys already prototyped this for ESC RPM at srmainwaring/ardupilot_gazebo-1
on wips/wip-dronecan. The joint state bridge follows the same pattern but publishes
actuator.Status instead of esc.Status.

---

## Schema abstraction

The library reads a config file at startup and builds the joint tree from it.
Controller code never sees robot-specific names or numbers. Same library runs on
ArduBiped_Proto (8 DOF, 6 active) and ROBOTIS OP3 (20 DOF) with only the config
file changing.

---

## What is in scope for GSoC

- AP_Biomimetic C++ library -- schema, primitives, IK solver, ZMP controller
- Wire humanoid joints into AP_Servo_Telem via DroneCAN actuator.Status
- joint_state_bridge.py -- Gazebo to DroneCAN bridge for sim
- Demonstrated on ROBOTIS OP3, ported to Unitree H1 in final two weeks
- SITL test cases and setup guide

Not in scope: arms, manipulation, RL policy layer, MAVLink waypoint navigation
(stretch goal only).

---

## References

- Rhys Mainwaring (2024). Sending Gazebo sensor data to ArduPilot using DroneCAN.
  discuss.ardupilot.org/t/sending-gazebo-sensor-data-to-ardupilot-using-dronecan/125730
- Bhajneet Singh Bedi GSoC 2025.
  discuss.ardupilot.org/t/gsoc-2025-wrapping-up-gazebo-plug-in-model-of-a-motor/138509
- GSoC 2020 Walking Robot Support.
  discuss.ardupilot.org/t/gsoc-2020-walking-robot-support-for-ardupilot-conclusion/61112
- AP_Servo_Telem:
  github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Servo_Telem/AP_Servo_Telem.h

## References update -- added ZMP

- Vukobratovic, M. and Borovac, B. (2004). Zero-Moment Point: Thirty-Five Years of Its Life.
  International Journal of Humanoid Robotics, 1(1), pp. 157-173.


---

## Implementation Phases

GSoC 2026 -- Neeta Misericordia
Added: 2026-06-10

Some parts of this system need to be correct from day one. Others can ship as a
working prototype as long as the function is clear, the component is decoupled, and
there is an obvious route to upgrade it later. This section maps each component to
one of those two categories.

Phase 1: working in SITL by end of GSoC.
Phase 2: post-GSoC upgrade with a documented path for a future contributor.

The interface must stay general and not be customized to any specific robot.
That applies in both phases.

---

### AP_Biomimetic core library

Phase: 1

This is the foundation. It has to be solid from day one because everything else
depends on it. The schema abstraction, the joint tree, the SRV_Channels output path,
and the AP_Servo_Telem read path all need to be correct and general before any balance
work can happen. No prototype shortcuts here.

Upgrade path in Phase 2: add support for torque control mode alongside position
control, once real hardware is in the loop and the DroneCAN actuator.Command torque
field can be tested.

---

### joint_state_bridge.py

Phase: 1 prototype, Phase 2 upgrade planned

The bridge works and is good enough to run the feedback loop in SITL. The joint
mapping and topic names are hardcoded. Rhys noted this is a known limitation and his
intention is to make it configurable via a YAML file, similar to how ros-gz-bridge
works, but targeting DroneCAN instead of ROS topics.

The prototype is acceptable for GSoC because: it is decoupled from AP_Biomimetic, the
function is obvious, and the hardcoding only lives in the bridge script. AP_Biomimetic
itself stays general.

Upgrade path in Phase 2: replace hardcoded joint list with a YAML config file. Follow
the pattern Rhys is developing in ardupilot_gazebo-1. New robots plug in by writing a
config file, no code changes needed.

---

### DroneCAN actuator.Status feedback path

Phase: 1

The interface choice (DroneCAN actuator.Status into AP_Servo_Telem) is the right
long-term design. It matches real hardware exactly and is decoupled from the physics
loop rate. This is not a prototype, this is the permanent interface. Implement it
correctly in Phase 1.

---

### ZMP/LIPM balance controller

Phase: 1 stub, Phase 2 full solver

The stub in balance_update() establishes the plumbing: reads CoM estimate from EKF3,
computes a ZMP correction, returns zero for now. The structure is there so the gait
planner has somewhere to call into.

The full DARE-precomputed LIPM solver from the Python prototype gets ported to C++
in Phase 1 (July milestone). That covers the GSoC demo.

Upgrade path in Phase 2: replace the fixed-horizon LIPM with a model predictive
controller. The MPC layer can be added above balance_update() without changing the
interface.

---

### Gait primitives

Phase: 1 prototype, Phase 2 upgrade planned

Static gait (fixed step sequence, no reactive adjustment) is enough for the GSoC
demo. Weight shift and foot sequencing work. The robot walks.

Upgrade path in Phase 2: add reactive gait that adjusts step timing and placement
based on live ZMP error. This is where the project gets interesting for a post-GSoC
contributor. The interface in AP_Biomimetic is already set up to support it.

---

### IK solver

Phase: 1

Analytic IK for a 6-DOF leg chain. Fixed at Phase 1 quality. No upgrade needed for
the scope of this project. If someone later adds arms or a different kinematic chain
they will write a new solver and register it, the existing leg solver does not change.

---

### Robot config file / schema abstraction

Phase: 1

This is what keeps the library general. The config file is what changes between
ArduBiped_Proto, ROBOTIS OP3, and Unitree H1. The library code never sees robot-
specific names. This has to be right in Phase 1 or every robot port becomes a code
change instead of a config change.

---

### SITL test cases and setup guide

Phase: 1, delivered in August

Automated checks for: joint state round-trip (command in, telemetry back), ZMP stays
inside support polygon during static stand, gait produces forward displacement. Setup
guide covers SIM_JSON bridge config, lockstep tuning, and known failure modes.

---

### MAVLink waypoint navigation (AUTO mode)

Phase: 2, stretch goal

The robot walks to a GPS waypoint under AUTO mode. Not in scope for GSoC. Mentioned
as a stretch goal. The ModeLegged class is already in the mode enum so the entry
point exists. Someone can build on top of it after GSoC.

---

### Arms and manipulation

Phase: 2, future GSoC

Out of scope for this project entirely. The schema abstraction already supports more
than 2 limbs so AP_Biomimetic does not need to change. A future contributor adds an
arm config file and an arm-specific gait primitive. The library stays the same.
