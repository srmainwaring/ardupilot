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
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <rosgraph_msgs/msg/clock.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/joy.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <std_msgs/msg/int32.h>
#include <tf2_msgs/msg/tf_message.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>
#include <AP_AHRS/AP_AHRS.h>

#include "fcntl.h"

#include <AP_Param/AP_Param.h>

// UDP only on SITL for now
#define AP_UROS_UDP_ENABLED 1 //(CONFIG_HAL_BOARD == HAL_BOARD_SITL)

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
    const unsigned int timer_timeout = 1000;

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
};

#endif // AP_UROS_ENABLED
