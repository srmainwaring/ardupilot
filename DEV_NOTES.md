
## AP Codebase Audit — 2026-06-03

### AP_ESC_Telem — closest analogy to joint feedback
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

### SIM_Gazebo — template for sim bridge
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
