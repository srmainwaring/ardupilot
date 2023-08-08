#pragma once

#if AP_UROS_ENABLED

// micro-xrce-dds
#include "uxr/client/client.h"

// micro-ROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS msgs
#include <builtin_interfaces/msg/time.h>
#include <geographic_msgs/msg/geo_pose_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <rosgraph_msgs/msg/clock.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/joy.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <tf2_msgs/msg/tf_message.h>

#include <micro_ros_utilities/type_utilities.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>
#include <AP_AHRS/AP_AHRS.h>

#include "fcntl.h"

#include <AP_Param/AP_Param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

// UDP only on SITL for now
#define AP_UROS_UDP_ENABLED 0 //(CONFIG_HAL_BOARD == HAL_BOARD_SITL)

#if AP_UROS_UDP_ENABLED
#include <AP_HAL/utility/Socket.h>
#endif

extern const AP_HAL::HAL& hal;

class AP_UROS_Client
{
private:

    AP_Int8 enabled;

    HAL_Semaphore csem;

    // micro-ROS
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_timer_t timer;
    const unsigned int timer_timeout_ms = 1;

    // publishers
    rcl_publisher_t battery_state_publisher;
    sensor_msgs__msg__BatteryState battery_state_msg;
    uint64_t last_battery_state_time_ms;
    micro_ros_utilities_memory_conf_t battery_state_conf;
    bool battery_state_mem_init = false;
    bool battery_state_pub_init = false;

    rcl_publisher_t clock_publisher;
    rosgraph_msgs__msg__Clock clock_msg;
    uint64_t last_clock_time_ms;
    bool clock_pub_init = false;

    rcl_publisher_t geo_pose_publisher;
    geographic_msgs__msg__GeoPoseStamped geo_pose_msg;
    uint64_t last_geo_pose_time_ms;
    micro_ros_utilities_memory_conf_t geo_pose_conf;
    bool geo_pose_mem_init = false;
    bool geo_pose_pub_init = false;

    rcl_publisher_t local_pose_publisher;
    geometry_msgs__msg__PoseStamped local_pose_msg;
    uint64_t last_local_pose_time_ms;
    micro_ros_utilities_memory_conf_t local_pose_conf;
    bool local_pose_mem_init = false;
    bool local_pose_pub_init = false;

    rcl_publisher_t local_twist_publisher;
    geometry_msgs__msg__TwistStamped local_twist_msg;
    uint64_t last_local_twist_time_ms;
    micro_ros_utilities_memory_conf_t local_twist_conf;
    bool local_twist_mem_init = false;
    bool local_twist_pub_init = false;

    rcl_publisher_t nav_sat_fix_publisher;
    sensor_msgs__msg__NavSatFix nav_sat_fix_msg;
    uint64_t last_nav_sat_fix_time_ms;
    micro_ros_utilities_memory_conf_t nav_sat_fix_conf;
    bool nav_sat_fix_mem_init = false;
    bool nav_sat_fix_pub_init = false;

    rcl_publisher_t time_publisher;
    builtin_interfaces__msg__Time time_msg;
    uint64_t last_time_time_ms;
    bool time_pub_init = false;

    // outgoing transforms
    rcl_publisher_t tx_static_transforms_publisher;
    tf2_msgs__msg__TFMessage tx_static_transforms_msg;
    uint64_t last_tx_static_transforms_time_ms;
    micro_ros_utilities_memory_conf_t tx_static_transforms_conf;
    bool tx_static_transforms_mem_init = false;
    bool tx_static_transforms_pub_init = false;

    // subscribers
    rcl_subscription_t rx_joy_subscriber;
    sensor_msgs__msg__Joy rx_joy_msg;
    micro_ros_utilities_memory_conf_t rx_joy_conf;
    bool rx_joy_mem_init = false;
    bool rx_joy_sub_init = false;

    // incoming transforms
    rcl_subscription_t rx_dynamic_transforms_subscriber;
    tf2_msgs__msg__TFMessage rx_dynamic_transforms_msg;
    micro_ros_utilities_memory_conf_t rx_dynamic_transforms_conf;
    bool rx_dynamic_transforms_mem_init = false;
    bool rx_dynamic_transforms_sub_init = false;

    // thread handle and singleton
    TaskHandle_t uros_task_handle;
    static AP_UROS_Client *_singleton;

    // publishers
    void update_topic(sensor_msgs__msg__BatteryState& msg);
    void update_topic(rosgraph_msgs__msg__Clock& msg);
    void update_topic(geographic_msgs__msg__GeoPoseStamped& msg);
    void update_topic(geometry_msgs__msg__PoseStamped& msg);
    void update_topic(geometry_msgs__msg__TwistStamped& msg);
    bool update_topic(sensor_msgs__msg__NavSatFix& msg);
    void update_topic(builtin_interfaces__msg__Time& msg);
    void update_topic(tf2_msgs__msg__TFMessage& msg);

    // subscribers
    static void on_joy_msg(const void * msgin, void *context);
    static void on_tf_msg(const void * msgin, void *context);

    static void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
    static void uros_thread(void *arg);

#if AP_UROS_UDP_ENABLED
    // functions for udp transport
    bool urosUdpInit();
    static bool udp_transport_open(uxrCustomTransport* transport);
    static bool udp_transport_close(uxrCustomTransport* transport);
    static size_t udp_transport_write(uxrCustomTransport* transport,
            const uint8_t* buf, size_t len, uint8_t* error);
    static size_t udp_transport_read(uxrCustomTransport* transport,
            uint8_t* buf, size_t len, int timeout, uint8_t* error);

    struct {
        AP_Int32 port;
        // UDP endpoint
        const char* ip = "127.0.0.1";
        // UDP Allocation
        uxrCustomTransport transport;
        SocketAPM *socket;
    } udp;
#endif

public:
    AP_UROS_Client();

    bool start(void);
    void main_loop(void);

    //! @brief Initialize the client.
    //! @return True on successful initialization, false on failure.
    bool init() WARN_IF_UNUSED;

    //! @brief Set up the client.
    //! @return True on successful creation, false on failure
    bool create() WARN_IF_UNUSED;

    //! @brief Parameter storage.
    static const struct AP_Param::GroupInfo var_info[];

    static AP_UROS_Client *get_singleton();
};

#endif // AP_UROS_ENABLED
