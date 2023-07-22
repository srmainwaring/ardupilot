#include <AP_HAL/AP_HAL_Boards.h>

#if AP_UROS_ENABLED

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

// publishers
rcl_publisher_t int32_publisher;
std_msgs__msg__Int32 int32_msg;
uint64_t last_int32_time_ms;
static constexpr uint16_t DELAY_INT32_TOPIC_MS = 1000;

rcl_publisher_t time_publisher;
builtin_interfaces__msg__Time time_msg;
uint64_t last_time_time_ms;
static constexpr uint16_t DELAY_TIME_TOPIC_MS = 10;

// subscribers
rcl_subscription_t vector3_subscriber;
geometry_msgs__msg__Vector3 vector3_msg;

void update_topic(std_msgs__msg__Int32& msg)
{
    msg.data++;
}

void update_topic(builtin_interfaces__msg__Time& msg)
{
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        utc_usec = AP_HAL::micros64();
    }
    msg.sec = utc_usec / 1000000ULL;
    msg.nanosec = (utc_usec % 1000000ULL) * 1000UL;
}

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

        if (cur_time_ms - last_int32_time_ms > DELAY_INT32_TOPIC_MS)
        {
            update_topic(time_msg);
            last_int32_time_ms = cur_time_ms;
            RCSOFTCHECK(rcl_publish(&int32_publisher, &int32_msg, NULL));
            // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
            //     "UROS: sent int32: %d", int32_msg.data);
        }

        if (cur_time_ms - last_time_time_ms > DELAY_TIME_TOPIC_MS)
        {
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

    RCSOFTCHECK(rcl_publisher_fini(&int32_publisher, &node));
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
        &int32_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "std_msgs_msg_Int32"));

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

    // create executor
    const size_t number_of_handles = 3;
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


