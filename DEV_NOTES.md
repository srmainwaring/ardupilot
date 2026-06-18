
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

2026-06-11

Spent the session tracking down why ArduPilotPlugin was sending empty JSON to ardurover with the OP3 in sim. Turned out to be a chain of things.

First confirmed the plugin works on this machine by running iris_runway.sdf. Got MasterIn 1856, EKF healthy. So the issue was always OP3 specific.

Tried fixing imuName, tried native SDF conversion with gz sdf -p, tried absolute mesh paths, tried removing conflicting plugins. Still empty JSON every time. Eventually checked gz model and saw the IMU sensor parent was robotis_op3 not base. Read the ArduPilotPlugin source -- line 1156 checks if the sensor parent entity is a Link type and aborts silently if it is not. That is what was happening.

To understand what a working sensor looks like I built a minimal box SDF -- one link, one clean imu sensor, ArduPilotPlugin, nothing else. Got MasterIn 7032 immediately. The working sensor block is:

    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>50</update_rate>
      <pose>0 0 0 0 0 0</pose>
      <imu/>
    </sensor>

No visualize tag, no topic tag, self-closing imu/ not open/close pair. URDF conversion adds extra tags that are not valid SDF and the converted sensor ends up mis-parented to the model root.

Applied the same clean structure to the OP3 converted SDF, embedded the model directly in the world file instead of using include, relaunched and got MasterIn 1922. OP3 is connected.

The sensor parent in gz model still shows model root not the link -- the plugin finds it via unscoped name fallback. It works but worth keeping an eye on.

models/ardupilot_box is now in ardupilot_gazebo-1 as a minimal working reference for any future model integration with ArduPilotPlugin.

Next: mode 20, joint_state_bridge.py, CSRV in DataFlash.

2026-06-12

Spent most of the session trying to verify CSRV (servo telemetry) shows up in the DataFlash log once the DroneCAN actuator.Status pipeline is running. Hit a chain of separate, unrelated problems that all looked like the same symptom at first.

First found that AP_SERVO_TELEM_ENABLED was compiling out the handle_actuator_status function entirely because NUM_SERVO_CHANNELS was not defined for the SITL board. Added NUM_SERVO_CHANNELS 16 to hwdef.dat and confirmed via nm that handle_actuator_status is now in the binary.

Then ran into severe WiFi interference. Running Gazebo plus ardurover plus the DroneCAN bridge at the same time killed my internet connection completely, needing a full process kill to recover. Traced it to gz-transport and AP both joining multicast groups on the WiFi interface instead of loopback. GZ_IP=127.0.0.1 fixed the Gazebo side. Tried patching Socket.cpp to force AP onto loopback too but it did not work cleanly and reverted it -- not a good permanent fix anyway, that file is core infrastructure.

Then discovered param show CSRV* was never going to show anything because CSRV is a DataFlash log message type, not a live parameter. Wasted a lot of time checking params for something that only exists in the BIN log.

Once checking the BIN log directly, found two more blockers stacked on top of each other. First, disk was at 96% full because of an old unused Docker image (open-webui ollama variant, 12.6GB, not even running) which was apparently causing ardurover to hang on log file operations. Removed the image, freed 12GB.

Second and the real blocker: LOG_DISARMED was 0, so ardurover was never writing any log data at all while disarmed, regardless of anything else being correct. Set LOG_DISARMED 1 and log writes started immediately.

Got a clean 30 second run with Gazebo, ardurover, and the bridge all running together, log file actively growing. Checked it for CSRV messages: zero. Checked MSG entries for any CAN node startup text: none. This means CAN_P1_DRIVER and CAN_D1_PROTOCOL had reset back to default after eeprom.bin was deleted earlier in the session to rule out a corrupted state -- so AP never actually had its DroneCAN driver enabled during that successful logging run.

So as of end of session: NUM_SERVO_CHANNELS fix confirmed compiled in, LOG_DISARMED fix confirmed working, but the actual CSRV verification with CAN enabled has not happened yet. Need to redo the full pipeline test with CAN_P1_DRIVER 1 and CAN_D1_PROTOCOL 1 set and saved before the next session.

Machine also had a stretch where ardurover would intermittently not respond to MAVProxy connection attempts despite the port being open. Never diagnosed the root cause, seemed to clear up on its own after some kills. Worth retesting after a reboot before assuming it is a real bug.

Lesson for future sessions: do not delete eeprom.bin casually, it resets every saved param including CAN config. If a fresh param state is genuinely needed, save the param list first with param save_logged_params or similar before clearing it.

2026-06-12
...


2026-06-13

Picked back up after the reboot. Set LOG_DISARMED, CAN_P1_DRIVER, and CAN_D1_PROTOCOL fresh since eeprom.bin holds these and they reset to default whenever it gets touched. Ran the full pipeline: Gazebo with op3_direct.sdf, ardurover, and joint_state_bridge.py all together for 30 seconds.

CSRV confirmed working. 12348 messages in the log, all 12 joint ids (0 through 11) present, real position data flowing, for example Id 0 Pos -1.15 degrees, Id 3 Pos 51.98 degrees. Force and Speed mostly read as qnan which is expected since the OP3 sim does not currently report torque or velocity through the bridge, only position. That is fine for now and matches what AP_Servo_Telem expects to receive incrementally.

This closes the full sim pipeline: Gazebo joint state to DroneCAN actuator.Status to AP_DroneCAN handle_actuator_status to AP_Servo_Telem to CSRV DataFlash log. Every link in that chain is now verified working end to end.

Next: AP_Biomimetic should start reading joint state via AP_Servo_Telem::get_telem() per joint index and feed it into balance_update(). That is the next real coding task.


2026-06-17

Verified AP_Biomimetic::_read_telem() actually pulls live joint data through the full pipeline. Added a temporary debug print on joint 0, switched to mode 20 (LEGGED), and watched pos go from 0.00 to -0.11 as real Gazebo data flowed through. valid flag was 1 throughout once data started arriving. Confirms the chain Gazebo to bridge to DroneCAN to AP_DroneCAN to AP_Servo_Telem to AP_Biomimetic._state[] is fully working, not just CSRV in isolation.

Removed the debug print, rebuilt, binary size matches the pre-debug build exactly so the removal was clean.

This means the read side of AP_Biomimetic is done and proven. get_joint_state() is ready for the gait planner and balance controller to consume real position data. Speed and torque are still expected to read as 0 or near it since the OP3 bridge currently only sends position, not velocity or force -- that matches what joint_state_bridge.py actually publishes today.

Next: decide whether to extend joint_state_bridge.py to publish velocity (it is available from /op3_gz_joint_states already, just not wired into the DroneCAN message yet) before moving on to gait primitives, or start gait work now with position-only feedback since that is enough for the July ZMP stub.
