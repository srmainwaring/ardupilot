
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
