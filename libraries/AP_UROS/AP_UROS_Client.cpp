#include <AP_HAL/AP_HAL_Boards.h>

#if AP_UROS_ENABLED

#include <algorithm>
#include <string.h>

#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_UROS_Client.h"

#define RCCHECK(fn) {\
    rcl_ret_t temp_rc = fn;\
    if ((temp_rc != RCL_RET_OK)) {\
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, \
            "UROS: failed status on line %d: %d", \
            __LINE__, (int)temp_rc);\
        return false;\
    } \
}

#define RCSOFTCHECK(fn) {\
    rcl_ret_t temp_rc = fn;\
    if ((temp_rc != RCL_RET_OK)) {\
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, \
            "UROS: failed status on line %d: %d. Continuing", \
            __LINE__, (int)temp_rc);\
    }\
}

// topic constants
// static char WGS_84_FRAME_ID[] = "WGS-84";
// https://www.ros.org/reps/rep-0105.html#base-link
// static char BASE_LINK_FRAME_ID[] = "base_link";

// publishers
rcl_publisher_t battery_state_publisher;
sensor_msgs__msg__BatteryState battery_state_msg;
uint64_t last_battery_state_time_ms;
static constexpr uint16_t DELAY_BATTERY_STATE_TOPIC_MS = 1000;

rcl_publisher_t clock_publisher;
rosgraph_msgs__msg__Clock clock_msg;
uint64_t last_clock_time_ms;
static constexpr uint16_t DELAY_CLOCK_TOPIC_MS = 10;

rcl_publisher_t local_pose_publisher;
geometry_msgs__msg__PoseStamped local_pose_msg;
uint64_t last_local_pose_time_ms;
static constexpr uint16_t DELAY_LOCAL_POSE_TOPIC_MS = 33;

rcl_publisher_t local_twist_publisher;
geometry_msgs__msg__TwistStamped local_twist_msg;
uint64_t last_local_twist_time_ms;
static constexpr uint16_t DELAY_LOCAL_TWIST_TOPIC_MS = 33;

rcl_publisher_t nav_sat_fix_publisher;
sensor_msgs__msg__NavSatFix nav_sat_fix_msg;
uint64_t last_nav_sat_fix_time_ms;
static constexpr uint16_t DELAY_NAV_SAT_FIX_TOPIC_MS = 1000;

rcl_publisher_t static_transform_publisher;
tf2_msgs__msg__TFMessage static_transform_msg;
uint64_t last_static_transform_time_ms;
static constexpr uint16_t DELAY_STATIC_TRANSFORM_TOPIC_MS = 1000;

rcl_publisher_t time_publisher;
builtin_interfaces__msg__Time time_msg;
uint64_t last_time_time_ms;
static constexpr uint16_t DELAY_TIME_TOPIC_MS = 10;

// subscribers
rcl_subscription_t vector3_subscriber;
geometry_msgs__msg__Vector3 vector3_msg;

// update published topics

void update_topic(builtin_interfaces__msg__Time& msg);

// implementation copied from:
// void AP_DDS_Client::update_topic(sensor_msgs_msg_BatteryState& msg, const uint8_t instance) 
void update_topic(sensor_msgs__msg__BatteryState& msg)
{
    const uint8_t instance = 0;

    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES) {
        return;
    }

    update_topic(msg.header.stamp);
    auto &battery = AP::battery();

    if (!battery.healthy(instance)) {
        msg.power_supply_status = 3; //POWER_SUPPLY_HEALTH_DEAD
        msg.present = false;
        return;
    }
    msg.present = true;

    msg.voltage = battery.voltage(instance);

    float temperature;
    msg.temperature = (battery.get_temperature(temperature, instance)) ? temperature : NAN;

    float current;
    msg.current = (battery.current_amps(current, instance)) ? -1 * current : NAN;

    const float design_capacity = (float)battery.pack_capacity_mah(instance) * 0.001;
    msg.design_capacity = design_capacity;

    uint8_t percentage;
    if (battery.capacity_remaining_pct(percentage, instance)) {
        msg.percentage = percentage * 0.01;
        msg.charge = (percentage * design_capacity) * 0.01;
    } else {
        msg.percentage = NAN;
        msg.charge = NAN;
    }

    msg.capacity = NAN;

    if (battery.current_amps(current, instance)) {
        if (percentage == 100) {
            msg.power_supply_status = 4;   //POWER_SUPPLY_STATUS_FULL
        } else if (current < 0.0) {
            msg.power_supply_status = 1;   //POWER_SUPPLY_STATUS_CHARGING
        } else if (current > 0.0) {
            msg.power_supply_status = 2;   //POWER_SUPPLY_STATUS_DISCHARGING
        } else {
            msg.power_supply_status = 3;   //POWER_SUPPLY_STATUS_NOT_CHARGING
        }
    } else {
        msg.power_supply_status = 0; //POWER_SUPPLY_STATUS_UNKNOWN
    }

    msg.power_supply_health = (battery.overpower_detected(instance)) ? 4 : 1; //POWER_SUPPLY_HEALTH_OVERVOLTAGE or POWER_SUPPLY_HEALTH_GOOD

    msg.power_supply_technology = 0; //POWER_SUPPLY_TECHNOLOGY_UNKNOWN

    // TODO(srmainwaring) - initialise cell_voltage sequence
    // if (battery.has_cell_voltages(instance)) {
    //     const uint16_t* cellVoltages = battery.get_cell_voltages(instance).cells;
    //     std::copy(cellVoltages, cellVoltages + AP_BATT_MONITOR_CELLS_MAX, msg.cell_voltage);
    // }
}

// implementation copied from:
// void AP_DDS_Client::update_topic(rosgraph_msgs_msg_Clock& msg)
void update_topic(rosgraph_msgs__msg__Clock& msg)
{
    update_topic(msg.clock);
}

// implementation copied from:
// void AP_DDS_Client::update_topic(geometry_msgs_msg_PoseStamped& msg)
void update_topic(geometry_msgs__msg__PoseStamped& msg)
{
    update_topic(msg.header.stamp);

    // TODO(srmainwaring) - initialise frame_id memory
    // strcpy(msg.header.frame_id, BASE_LINK_FRAME_ID);

    auto &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    // ROS REP 103 uses the ENU convention:
    // X - East
    // Y - North
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // AP_AHRS uses the NED convention
    // X - North
    // Y - East
    // Z - Down
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z

    Vector3f position;
    if (ahrs.get_relative_position_NED_home(position)) {
        msg.pose.position.x = position[1];
        msg.pose.position.y = position[0];
        msg.pose.position.z = -position[2];
    }

    // In ROS REP 103, axis orientation uses the following convention:
    // X - Forward
    // Y - Left
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z (NED to ENU convertion) as well as a 90 degree rotation in the Z axis
    // for x to point forward
    Quaternion orientation;
    if (ahrs.get_quaternion(orientation)) {
        Quaternion aux(orientation[0], orientation[2], orientation[1], -orientation[3]); //NED to ENU transformation
        Quaternion transformation (sqrtF(2) * 0.5,0,0,sqrtF(2) * 0.5); // Z axis 90 degree rotation
        orientation = aux * transformation;
        msg.pose.orientation.w = orientation[0];
        msg.pose.orientation.x = orientation[1];
        msg.pose.orientation.y = orientation[2];
        msg.pose.orientation.z = orientation[3];
    }
}

// implementation copied from:
// void AP_DDS_Client::update_topic(geometry_msgs_msg_TwistStamped& msg)
void update_topic(geometry_msgs__msg__TwistStamped& msg)
{
    update_topic(msg.header.stamp);

    // TODO(srmainwaring) - initialise frame_id memory
    // strcpy(msg.header.frame_id, BASE_LINK_FRAME_ID);

    auto &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    // ROS REP 103 uses the ENU convention:
    // X - East
    // Y - North
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // AP_AHRS uses the NED convention
    // X - North
    // Y - East
    // Z - Down
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        msg.twist.linear.x = velocity[1];
        msg.twist.linear.y = velocity[0];
        msg.twist.linear.z = -velocity[2];
    }

    // In ROS REP 103, axis orientation uses the following convention:
    // X - Forward
    // Y - Left
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // The gyro data is received from AP_AHRS in body-frame
    // X - Forward
    // Y - Right
    // Z - Down
    // As a consequence, to follow ROS REP 103, it is necessary to invert Y and Z
    Vector3f angular_velocity = ahrs.get_gyro();
    msg.twist.angular.x = angular_velocity[0];
    msg.twist.angular.y = -angular_velocity[1];
    msg.twist.angular.z = -angular_velocity[2];
}

// implementation copied from:
void update_topic(tf2_msgs__msg__TFMessage& msg)
{
}

// implementation copied from:
// bool AP_DDS_Client::update_topic(sensor_msgs_msg_NavSatFix& msg, const uint8_t instance)
// TODO(srmainwaring) - this version always provides an update - which may be valid;
//                      update scheduling to match AP_DDS
void update_topic(sensor_msgs__msg__NavSatFix& msg)
{
    const uint8_t instance = 0;

    // Add a lambda that takes in navsatfix msg and populates the cov
    // Make it constexpr if possible
    // https://www.fluentcpp.com/2021/12/13/the-evolutions-of-lambdas-in-c14-c17-and-c20/
    // constexpr auto times2 = [] (sensor_msgs_msg_NavSatFix* msg) { return n * 2; };

    // assert(instance >= GPS_MAX_RECEIVERS);
    if (instance >= GPS_MAX_RECEIVERS) {
        // return false;
        return;
    }

    auto &gps = AP::gps();
    WITH_SEMAPHORE(gps.get_semaphore());

    if (!gps.is_healthy(instance)) {
        msg.status.status = -1; // STATUS_NO_FIX
        msg.status.service = 0; // No services supported
        msg.position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
        // return false;
        return;
    }

    // No update is needed
    const auto last_fix_time_ms = gps.last_fix_time_ms(instance);
    if (last_nav_sat_fix_time_ms == last_fix_time_ms) {
        // return false;
        return;
    } else {
        last_nav_sat_fix_time_ms = last_fix_time_ms;
    }


    update_topic(msg.header.stamp);

    // TODO(srmainwaring) - initialise frame_id memory
    // strcpy(msg.header.frame_id, WGS_84_FRAME_ID);
    msg.status.service = 0; // SERVICE_GPS
    msg.status.status = -1; // STATUS_NO_FIX


    //! @todo What about glonass, compass, galileo?
    //! This will be properly designed and implemented to spec in #23277
    msg.status.service = 1; // SERVICE_GPS

    const auto status = gps.status(instance);
    switch (status) {
    case AP_GPS::NO_GPS:
    case AP_GPS::NO_FIX:
        msg.status.status = -1; // STATUS_NO_FIX
        msg.position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
        // return true;
        return;
    case AP_GPS::GPS_OK_FIX_2D:
    case AP_GPS::GPS_OK_FIX_3D:
        msg.status.status = 0; // STATUS_FIX
        break;
    case AP_GPS::GPS_OK_FIX_3D_DGPS:
        msg.status.status = 1; // STATUS_SBAS_FIX
        break;
    case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:
    case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
        msg.status.status = 2; // STATUS_SBAS_FIX
        break;
    default:
        //! @todo Can we not just use an enum class and not worry about this condition?
        break;
    }
    const auto loc = gps.location(instance);
    msg.latitude = loc.lat * 1E-7;
    msg.longitude = loc.lng * 1E-7;

    int32_t alt_cm;
    if (!loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
        // With absolute frame, this condition is unlikely
        msg.status.status = -1; // STATUS_NO_FIX
        msg.position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
        // return true;
        return;
    }
    msg.altitude = alt_cm * 0.01;

    // ROS allows double precision, ArduPilot exposes float precision today
    Matrix3f cov;
    msg.position_covariance_type = (uint8_t)gps.position_covariance(instance, cov);
    msg.position_covariance[0] = cov[0][0];
    msg.position_covariance[1] = cov[0][1];
    msg.position_covariance[2] = cov[0][2];
    msg.position_covariance[3] = cov[1][0];
    msg.position_covariance[4] = cov[1][1];
    msg.position_covariance[5] = cov[1][2];
    msg.position_covariance[6] = cov[2][0];
    msg.position_covariance[7] = cov[2][1];
    msg.position_covariance[8] = cov[2][2];

    // return true;
}

// implementation copied from:
// void AP_DDS_Client::update_topic(builtin_interfaces_msg_Time& msg)
void update_topic(builtin_interfaces__msg__Time& msg)
{
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        utc_usec = AP_HAL::micros64();
    }
    msg.sec = utc_usec / 1000000ULL;
    msg.nanosec = (utc_usec % 1000000ULL) * 1000UL;
}

// subscriber callbacks
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    // WITH_SEMAPHORE(csem);

    // get the timer clock
    rcl_clock_t * clock;
    RCSOFTCHECK(rcl_timer_clock(timer, &clock));

    // TODO(srmainwaring) - use the rcl_clock...
    const auto cur_time_ms = AP_HAL::millis64();

    if (timer != NULL) {

        if (cur_time_ms - last_battery_state_time_ms > DELAY_BATTERY_STATE_TOPIC_MS) {
            update_topic(battery_state_msg);
            last_battery_state_time_ms = cur_time_ms;
            RCSOFTCHECK(rcl_publish(&battery_state_publisher, &battery_state_msg, NULL));
        }

        if (cur_time_ms - last_clock_time_ms > DELAY_CLOCK_TOPIC_MS) {
            update_topic(clock_msg);
            last_clock_time_ms = cur_time_ms;
            RCSOFTCHECK(rcl_publish(&clock_publisher, &clock_msg, NULL));
        }

        if (cur_time_ms - last_local_pose_time_ms > DELAY_LOCAL_POSE_TOPIC_MS) {
            update_topic(local_pose_msg);
            last_local_pose_time_ms = cur_time_ms;
            RCSOFTCHECK(rcl_publish(&local_pose_publisher, &local_pose_msg, NULL));
        }

        if (cur_time_ms - last_local_twist_time_ms > DELAY_LOCAL_TWIST_TOPIC_MS) {
            update_topic(local_twist_msg);
            last_local_twist_time_ms = cur_time_ms;
            RCSOFTCHECK(rcl_publish(&local_twist_publisher, &local_twist_msg, NULL));
        }

        if (cur_time_ms - last_nav_sat_fix_time_ms > DELAY_NAV_SAT_FIX_TOPIC_MS) {
            update_topic(nav_sat_fix_msg);
            last_nav_sat_fix_time_ms = cur_time_ms;
            RCSOFTCHECK(rcl_publish(&nav_sat_fix_publisher, &nav_sat_fix_msg, NULL));
        }

        if (cur_time_ms - last_static_transform_time_ms > DELAY_STATIC_TRANSFORM_TOPIC_MS) {
            update_topic(static_transform_msg);
            last_static_transform_time_ms = cur_time_ms;
            RCSOFTCHECK(rcl_publish(&static_transform_publisher, &static_transform_msg, NULL));
        }

        if (cur_time_ms - last_time_time_ms > DELAY_TIME_TOPIC_MS) {
            update_topic(time_msg);
            last_time_time_ms = cur_time_ms;
            RCSOFTCHECK(rcl_publish(&time_publisher, &time_msg, NULL));
            // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
            //     "UROS: sent time: %d, %d", time_msg.sec, time_msg.nanosec);
        }
    }
}

void subscription_callback(const void * msgin)
{
    const geometry_msgs__msg__Vector3 * msg =
        (const geometry_msgs__msg__Vector3 *)msgin;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UROS: x: %f, y: %f, z: %f",
        msg->x, msg->y, msg->z);
}

const AP_Param::GroupInfo AP_UROS_Client::var_info[] {

    // @Param: _ENABLE
    // @DisplayName: UROS enable
    // @Description: Enable UROS subsystem
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_UROS_Client, enabled, 0,
        AP_PARAM_FLAG_ENABLE),

#if AP_UROS_UDP_ENABLED
    // @Param: _UDP_PORT
    // @DisplayName: UROS UDP port
    // @Description: UDP port number for UROS
    // @Range: 1 65535
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_PORT", 2, AP_UROS_Client, udp.port, 2019),
#endif

    AP_GROUPEND
};

/*
  start the UROS thread
 */
bool AP_UROS_Client::start(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::load_object_from_eeprom(this, var_info);

    if (enabled == 0) {
        return true;
    }

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_UROS_Client::main_loop, void),
            "UROS", 8192, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"UROS: thread create failed");
        return false;
    }
    return true;
}

/*
  main loop for UROS thread
 */
void AP_UROS_Client::main_loop(void)
{
    if (!init() || !create()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UROS: creation failed");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UROS: initialization passed");

    // one-time actions

    // periodic actions
    rclc_executor_spin(&executor);

    RCSOFTCHECK(rcl_subscription_fini(&vector3_subscriber, &node));

    RCSOFTCHECK(rcl_publisher_fini(&battery_state_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&clock_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&local_pose_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&local_twist_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&nav_sat_fix_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&static_transform_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&time_publisher, &node));

    RCSOFTCHECK(rcl_node_fini(&node));
}

bool AP_UROS_Client::init()
{
    // initialize transport
    bool initTransportStatus = false;

#if AP_UROS_UDP_ENABLED
    // fallback to UDP if available
    if (!initTransportStatus) {
        initTransportStatus = urosUdpInit();
    }
#endif

    if (initTransportStatus) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UROS: transport initializated");
    }
    else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
            "UROS: transport initialization failed");
        return false;
    }

    // create allocator
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(
        &node, "ardupilot_uros", "", &support));

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UROS: init complete");

    return true;
}

bool AP_UROS_Client::create()
{
    WITH_SEMAPHORE(csem);

    // create publishers
    RCCHECK(rclc_publisher_init_default(
        &battery_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        "ap/battery/battery0"));

    RCCHECK(rclc_publisher_init_default(
        &clock_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rosgraph_msgs, msg, Clock),
        "ap/clock"));

    RCCHECK(rclc_publisher_init_default(
        &local_pose_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        "ap/pose/filtered"));

    RCCHECK(rclc_publisher_init_default(
        &local_twist_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        "ap/twist/filtered"));

    RCCHECK(rclc_publisher_init_default(
        &nav_sat_fix_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "ap/navsat/navsat0"));

    RCCHECK(rclc_publisher_init_default(
        &static_transform_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        "ap/tf_static"));

    RCCHECK(rclc_publisher_init_default(
        &time_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(builtin_interfaces, msg, Time),
        "ap/time"));

    // create subscribers
    RCCHECK(rclc_subscription_init_default(
        &vector3_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "geometry_msgs_msg_Vector3"));

    // create timer
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout_ms),
        timer_callback));

    // number of entities
    constexpr size_t number_of_publishers = 7;
    constexpr size_t number_of_subscribers = 1;
    constexpr size_t number_of_handles =
        number_of_publishers + number_of_subscribers;

    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context,
        number_of_handles, &allocator));

  	RCCHECK(rclc_executor_add_timer(&executor, &timer));

    RCCHECK(rclc_executor_add_subscription(&executor, &vector3_subscriber,
        &vector3_msg, &subscription_callback, ON_NEW_DATA));

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UROS: create complete");

    return true;
}

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
extern "C" {
    int clock_gettime(clockid_t clockid, struct timespec *ts);
}

int clock_gettime(clockid_t clockid, struct timespec *ts)
{
    //! @todo the value of clockid is ignored here.
    //! A fallback mechanism is employed against the caller's choice of clock.
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        utc_usec = AP_HAL::micros64();
    }
    ts->tv_sec = utc_usec / 1000000ULL;
    ts->tv_nsec = (utc_usec % 1000000ULL) * 1000UL;
    return 0;
}
#endif // CONFIG_HAL_BOARD

#endif // AP_UROS_ENABLED


