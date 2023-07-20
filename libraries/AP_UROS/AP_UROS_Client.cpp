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

// micro-ROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/vector3.h>

#define RCCHECK(fn) {\
  rcl_ret_t temp_rc = fn;\
  if ((temp_rc != RCL_RET_OK)) {\
    printf("Failed status on line %d: %d. Aborting.\n", \
        __LINE__, (int)temp_rc);\
  } \
}

#define RCSOFTCHECK(fn) {\
  rcl_ret_t temp_rc = fn;\
  if ((temp_rc != RCL_RET_OK)) {\
    printf("Failed status on line %d: %d. Continuing.\n", \
        __LINE__, (int)temp_rc);\
  }\
}

rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3 msg;

void subscription_callback(const void * msgin)
{
    const geometry_msgs__msg__Vector3 * pmsg =
        (const geometry_msgs__msg__Vector3 *)msgin;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"UROS: x: %f, y: %f, z: %f",
        pmsg->x, pmsg->y, pmsg->z);
}

const AP_Param::GroupInfo AP_UROS_Client::var_info[] {

    // @Param: _ENABLE
    // @DisplayName: UROS enable
    // @Description: Enable UROS subsystem
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_UROS_Client, enabled, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

/*
  start the DDS thread
 */
bool AP_UROS_Client::start(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::load_object_from_eeprom(this, var_info);

    if (enabled == 0) {
        return true;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_UROS_Client::main_loop, void),
                                      "UROS",
                                      8192, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
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
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"UROS: creation failed");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"UROS: initialization passed");

    // one-time actions

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(
        &node, "vector3_subscriber_rclc", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "geometry_msgs_msg_Vector3"));

    // create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    rclc_executor_spin(&executor);

    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    // periodic actions
    while (true) {
        hal.scheduler->delay(1);
        update();
    }
}

bool AP_UROS_Client::init()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"UROS: init complete");

    return true;
}

bool AP_UROS_Client::create()
{
    WITH_SEMAPHORE(csem);

   GCS_SEND_TEXT(MAV_SEVERITY_INFO,"UROS: create complete");

    return true;
}

void AP_UROS_Client::update()
{
    WITH_SEMAPHORE(csem);
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


