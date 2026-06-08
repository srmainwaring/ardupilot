
## Update -- 2026-06-08

AP_Servo_Telem already exists and has all fields needed:
- measured_position (degrees) -- joint position
- speed (degrees/sec) -- joint velocity  
- force (Nm) -- joint torque

No new library needed. Wire DroneCAN actuator.Status into
AP_Servo_Telem.update_telem_data() per joint index.
AP_Biomimetic reads joint state via AP_Servo_Telem.get_telem().
Already logs to DataFlash via write_log() at 10 Hz.
