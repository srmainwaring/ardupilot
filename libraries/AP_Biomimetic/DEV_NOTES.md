# AP_Biomimetic Dev Notes
## Architecture
### Command path (SITL)
AP_Biomimetic C++ -> SRV_Channels PWM -> ArduPilotPlugin -> Gazebo JPC -> robot joints
### Telemetry path (SITL + hardware ready)
Gazebo joint_states -> joint_state_bridge.py -> DroneCAN actuator.Status -> AP_Servo_Telem -> AP_Biomimetic C++
### Why command path uses PWM not DroneCAN
DroneCAN actuator.Command for the command side was raised by Rhys on 2026-06-08
but left as an open research question ("some things to think about there").
PWM via ArduPilotPlugin is the working SITL solution.
DroneCAN actuator.Command bridge for real hardware is documented future work.
### Why no ROS2 in the loop
AP_Biomimetic talks only to ArduPilot native systems: AP_Servo_Telem, SRV_Channels,
AP_AHRS. Gazebo handles physics internally via ArduPilotPlugin. No ROS2 middleware
in the control loop. ROS2 is only relevant for real hardware servo bus drivers,
which is future work beyond GSoC scope.
### Why 3+ steps is possible without ROS2
In SITL, Gazebo is the robot. ArduPilotPlugin handles the PWM to joint command
conversion internally. No ROS2 in the loop at all. 3+ consecutive steps is purely
a gait tuning problem in SITL.
## Hardware path (future work)
- DroneCAN actuator.Command bridge to replace PWM command path
- Rhys YAML-configurable DroneCAN bridge when ready
- Real servo bus integration (Dynamixel for OP3, Unitree SDK for H1)
- ROS2 drivers needed for real hardware only, not SITL
## What is proven (2026-07-08)
- Full pipeline end to end in SITL confirmed
- 12-joint DroneCAN telemetry flowing via AP_Servo_Telem
- AP_Biomimetic stand() and gait_step() running inside ArduPilot Rover SITL
- Robot takes alternating steps under full ArduPilot control
- Dataflash CSRV logs confirmed for all 12 joints
- Midterm blog post published on ArduPilot Discourse
- Both branches pushed and public
## Key decisions and who validated them
- DroneCAN actuator.Status for telemetry: agreed with Rhys 2026-06-08
- AP_Servo_Telem as the AP-side interface: Rhys confirmed 2026-06-08
- Hardcoded joint mapping as prototype: Rhys approved 2026-06-10
- Overall direction sensible and would work for sim and hardware: Rhys 2026-06-08
## Open questions
- DroneCAN command path for hardware: how to maintain lockstep (Rhys, 2026-06-08)
- YAML-configurable bridge: Rhys future work, upgrade path for joint_state_bridge.py
- H1 port: August scope, same AP_Biomimetic library, different SDF and bridge config
## Gait tuning state (2026-07-08)
- hip_roll lateral shift: 12 deg sinusoidal per leg phase
- gait_period: 1.5s
- step_len_deg: 15
- step_height_deg: 20
- ankle compensation: hip_pitch * 0.5 stance, -knee * 0.3 swing
- spawn lean: 0.12 rad pitch
- Result: alternating steps, falls before 3+ consecutive steps
- Next: tune hip_roll timing and magnitude for lateral balance
## Architecture is NOT contradicted by ROS2 question
Rhys said June 8: sensible and would work for sim and hardware.
Telemetry half is fully DroneCAN. Command half scales to hardware via
DroneCAN actuator.Command bridge. Route exists, not built yet.
Known and documented scope boundary, not a flaw.
