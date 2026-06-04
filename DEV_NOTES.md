
## AP Codebase Audit -- 2026-06-03

### AP_ESC_Telem -- closest analogy to joint feedback
- Lives in libraries/AP_ESC_Telem/
- Backend/frontend split: backend drivers call update_rpm() / update_telem_data()
- Frontend exposes get_rpm(), get_current() etc to rest of AP
- Sends to MAVLink via send_esc_telemetry_mavlink()
- Indexed by esc_index up to NUM_SERVO_CHANNELS

### Proposed AP_Joint_Telem library (new)
- Same backend/frontend pattern as AP_ESC_Telem
- TelemetryData struct fields: position (rad), velocity (rad/s), torque (Nm)
- In sim: SIM_Gazebo bridge calls update_joint_telem() from /joint_states
- On real hardware: DroneCAN or MAVLink backend calls same function
- AP_Biomimetic reads joint state via get_joint_position() etc

### SIM_Gazebo -- template for sim bridge
- libraries/SITL/SIM_Gazebo.h/.cpp
- Next step: read this to understand the existing sim data path into AP

## AP Codebase Audit -- 2026-06-03

### AP_ESC_Telem -- closest analogy to joint feedback
- Lives in libraries/AP_ESC_Telem/
- Backend/frontend split: backend drivers call update_rpm() / update_telem_data()
- Frontend exposes get_rpm(), get_current() etc to rest of AP
- Sends to MAVLink via send_esc_telemetry_mavlink()
- Indexed by esc_index up to NUM_SERVO_CHANNELS

### Proposed AP_Joint_Telem library (new)
- Same backend/frontend pattern as AP_ESC_Telem
- TelemetryData struct fields: position (rad), velocity (rad/s), torque (Nm)
- In sim: SIM_Gazebo bridge calls update_joint_telem() from /joint_states
- On real hardware: DroneCAN or MAVLink backend calls same function
- AP_Biomimetic reads joint state via get_joint_position() etc

### SIM_Gazebo -- template for sim bridge
- libraries/SITL/SIM_Gazebo.h/.cpp
- Next step: read this to understand the existing sim data path into AP

## SIM_Gazebo Data Path Audit -- 2026-06-03

### How the existing bridge works
- Single UDP socket between AP and Gazebo
- AP sends servo_packet: float motor_speed[16]
- Gazebo sends fdm_packet: IMU + velocity + position, NO joint state
- recv_fdm() unpacks fdm_packet into AP internal state each cycle

### What needs to change for humanoid joint feedback

Step 1 -- extend fdm_packet in SIM_Gazebo.h:
  double joint_position[32];   // rad
  double joint_velocity[32];   // rad/s
  double joint_torque[32];     // Nm

Step 2 -- extend servo_packet for more than 16 joints:
  float motor_speed[32];

Step 3 -- in recv_fdm() after last_timestamp = pkt.timestamp:
  call AP_Joint_Telem::update() with the new joint arrays

Step 4 -- Gazebo side plugin reads /joint_states and packs into fdm_packet

### Why UDP works here
- Same socket, same cycle, just bigger structs
- No new transport needed for sim
- Real hardware uses different backend (DroneCAN or MAVLink) but same AP_Joint_Telem frontend

## Correction -- 2026-06-03

### SIM_JSON not SIM_Gazebo
Previous audit referenced SIM_Gazebo and fdm_packet. This was wrong.
The correct interface used by ardupilot_gazebo is SIM_JSON with ArduPilotPlugin.
- AP side: libraries/SITL/SIM_JSON.h
- Gazebo side: ArduPilotPlugin in ardupilot_gazebo repo
- Communication: UDP, AP sends PWM binary packet, Gazebo replies with JSON payload
- Lockstepped so both sides stay in sync

### Rhys DroneCAN prototype -- already exists
Rhys built a working proof of concept for joint state feedback via DroneCAN:
- repo: srmainwaring/ardupilot_gazebo-1 branch wips/wip-dronecan
- uses JointStatePublisher plugin in Gazebo
- Python script bridges gz.msgs joint velocities to DroneCAN ESC status messages
- AP receives via existing DroneCAN ESC backends
- This is the template for AP_Joint_Telem feedback in sim

### What needs to happen next
- Clone srmainwaring/ardupilot_gazebo-1 wips/wip-dronecan and study it
- Check libraries/SITL/SIM_JSON.h to understand extensible JSON fields
- Joint feedback follows same pattern as rotor RPM: JointStatePublisher in Gazebo,
  Python bridge to DroneCAN, AP receives via DroneCAN backend

## SIM_JSON Deep Audit -- 2026-06-03

### Key findings from SIM_JSON.h
- Uses extensible keytable parser, not fixed binary struct like SIM_Gazebo
- Adding joint state to JSON would just need new keytable rows
- Already supports servo_packet_32 (32 channels, enough for OP3 20 DOF)
- State struct has rng, rc, battery, wind -- pattern for adding joints is clear
- Runs lockstepped at physics rate (~1000 Hz)

### Two options now clear for joint feedback in sim

Option A -- Extend SIM_JSON keytable:
  Add joint_pos[], joint_vel[], joint_tor[] to state struct
  Add rows to keytable for each joint
  Gazebo ArduPilotPlugin packs joint state into JSON reply
  Pro: single transport, lockstepped, no extra process
  Con: runs at 1000 Hz whether you want it or not

Option B -- Rhys DroneCAN bridge (already prototyped):
  SIM_JSON stays unchanged for flight dynamics
  JointStatePublisher in Gazebo publishes joint state
  Python script bridges to DroneCAN messages
  AP receives via existing DroneCAN backends
  Pro: decoupled rate, matches real hardware transport exactly
  Con: extra process, more moving parts in sim

### Recommendation
Option B for real hardware parity.
Option A as a fast fallback if DroneCAN setup is too complex for sim.
Both use the same AP_Joint_Telem frontend -- only the backend changes.
