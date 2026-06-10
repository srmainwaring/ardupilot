
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

## 2026-06-09

Today was a big day. Rhys went ahead and built a working prototype on
wips/wip-dronecan-extra showing DroneCAN actuator.Status messages flowing
from Gazebo gimbal servos all the way into AP DataFlash logs as CSRV.
He also caught that actuator_id indexes from 1 not 0 which would have
cost me hours to debug. Really grateful he did all that.

On my end I confirmed the OP3 joint topic is /op3_gz_joint_states and
it publishes gz.msgs.Model which is exactly what the JointStatesConverter
class expects. Got all 20 joint names from the live topic.

Wrote joint_state_bridge.py for the OP3 -- subscribes to
/op3_gz_joint_states, reads position and velocity for all 12 leg joints
by name, publishes actuator.Status per joint via DroneCAN at 50 Hz.
actuator_id 1-12 for l_hip_yaw through r_ank_roll. Committed to
Neetagrg/ardupilot_gazebo-1 on branch wip-op3-joint-bridge.

Also confirmed AP_Servo_Telem already has everything needed --
measured_position, speed, force -- so no new library required.
AP_Biomimetic just calls get_telem() per joint index.

Next: build AP_Biomimetic C++ skeleton and test the full pipeline with
AP SITL + OP3 sim + bridge all running together. Check CSRV in
mavexplorer.

## 2026-06-09

Today was a big day. Rhys went ahead and built a working prototype on
wips/wip-dronecan-extra showing DroneCAN actuator.Status messages flowing
from Gazebo gimbal servos all the way into AP DataFlash logs as CSRV.
He also caught that actuator_id indexes from 1 not 0 which would have
cost me hours to debug. Really grateful he did all that.

On my end I confirmed the OP3 joint topic is /op3_gz_joint_states and
it publishes gz.msgs.Model which is exactly what the JointStatesConverter
class expects. Got all 20 joint names from the live topic.

Wrote joint_state_bridge.py for the OP3 -- subscribes to
/op3_gz_joint_states, reads position and velocity for all 12 leg joints
by name, publishes actuator.Status per joint via DroneCAN at 50 Hz.
actuator_id 1-12 for l_hip_yaw through r_ank_roll. Committed to
Neetagrg/ardupilot_gazebo-1 on branch wip-op3-joint-bridge.

Also confirmed AP_Servo_Telem already has everything needed --
measured_position, speed, force -- so no new library required.
AP_Biomimetic just calls get_telem() per joint index.

Next: build AP_Biomimetic C++ skeleton and test the full pipeline with
AP SITL + OP3 sim + bridge all running together. Check CSRV in
mavexplorer.
Here are the dev notes for today covering everything that actually got built and fixed, not the handoff context you already had:

---

##  2026-06-10

**DESIGN.md -- Implementation Phases**

Wrote and pushed the Implementation Phases section Rhys requested. Maps every component to Phase 1 (working in SITL by end of GSoC) or Phase 2 (post-GSoC upgrade with documented path). Key decisions captured: joint_state_bridge.py hardcoding is accepted as Phase 1 prototype because it is decoupled and the function is obvious -- the upgrade path is YAML config following the pattern Rhys is developing in ardupilot_gazebo-1. DroneCAN actuator.Status path is marked Phase 1 permanent, not prototype -- that is the right long-term interface and needs to be correct from the start.

Cleaned up the intro after first draft removed "Rhys flagged this in the Discord" and "the key rule Rhys gave" phrasing. Design docs record decisions

Updated document date from 2026-06-08 to 2026-06-10.

Commits: `c29512bb24` (section added), `b513dba9e6` (intro tightened, date updated).

---

**AP_Biomimetic::balance_update() -- ZMP/LIPM stub**

Replaced the empty TODO stub with working plumbing for the July milestone.

What it does now: reads pitch from AHRS via `AP::ahrs().get_pitch()`, multiplies by proportional gain `kp = 1.0f` to get an ankle correction in degrees, clamps to +/- 10 deg so it cannot fight the stand targets, applies symmetrically to left ankle (joint 4) and right ankle (joint 10). Gated on `p_balance_enable == 1` so it does nothing until explicitly enabled.

Added `AP_AHRS/AP_AHRS.h` include to AP_Biomimetic.cpp.

Build confirmed clean: `bin/ardurover 4031694B` text, up 176 bytes from `4031518B`.

What slots in here in July: replace `kp = 1.0f` with DARE-precomputed LIPM gain, add lateral (roll) axis correction, add ZMP inside support polygon check before applying correction.

Joint index reference for this file: per side -- hip_roll=0, hip_yaw=1, hip_pitch=2, knee=3, ank_pitch=4, ank_roll=5. Left side is joints 0-5, right side is joints 6-11. Left ankle = 4, right ankle = 10.

---

**CSRV verification -- not done yet**

Every attempt hit an empty BIN because ardurover was launched before Gazebo. Next session must run Terminal 1 (Gazebo) first, wait for it to be fully up, then Terminal 2 (SITL). The BIN only gets data when the JSON interface has something to connect to.
