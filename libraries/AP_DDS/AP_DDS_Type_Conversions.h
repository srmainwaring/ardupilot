// Class for handling type conversions for DDS.

#pragma once

#if AP_DDS_ENABLED

#include <AP_ROS/AP_ROS_TypeConversions.h>

#include "builtin_interfaces/msg/Time.h"
#include "geometry_msgs/msg/TransformStamped.h"
#include "tf2_msgs/msg/TFMessage.h"

class AP_DDS_Type_Conversions
{
public:

    // Convert ROS time to a uint64_t [μS]
    static uint64_t time_u64_micros(const builtin_interfaces_msg_Time& ros_time);
};

// string specialisations
template <>
const char* string_data(const char* str) {
    return str;
}

// transform specialisations
template <>
uint32_t transforms_size(const tf2_msgs_msg_TFMessage& msg) {
    return msg.transforms_size;
}

template <>
struct transforms_type<tf2_msgs_msg_TFMessage>{
    typedef const geometry_msgs_msg_TransformStamped* type;
};

template <>
typename transforms_type<tf2_msgs_msg_TFMessage>::type
transforms_data(const tf2_msgs_msg_TFMessage& msg) {
    return msg.transforms;
}

#endif // AP_DDS_ENABLED
