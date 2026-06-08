# AP_Biomimetic Design Document
GSoC 2026 -- Neeta Misericordia
Updated: 2026-06-08

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
