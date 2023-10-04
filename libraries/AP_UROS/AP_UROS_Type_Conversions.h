// Class for handling type conversions for UROS.

#pragma once

#if AP_UROS_ENABLED

#include <AP_ROS/AP_ROS_TypeConversions.h>

#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

class AP_UROS_Type_Conversions
{
public:

    // Convert ROS time to a uint64_t [μS]
    static uint64_t time_u64_micros(const builtin_interfaces__msg__Time& ros_time);
};

// string specialisations
template <>
const char* string_data(const rosidl_runtime_c__String& str);

template <>
char* mutable_string_data(rosidl_runtime_c__String& str);

// transform specialisations
template <>
struct transforms_size_type<tf2_msgs__msg__TFMessage> {
    typedef size_t type;
};

template <>
struct mutable_transforms_size_type<tf2_msgs__msg__TFMessage> {
    typedef size_t& type;
};

template <>
typename transforms_size_type<tf2_msgs__msg__TFMessage>::type
transforms_size(const tf2_msgs__msg__TFMessage& msg);

template <>
typename mutable_transforms_size_type<tf2_msgs__msg__TFMessage>::type
mutable_transforms_size(tf2_msgs__msg__TFMessage& msg);

template <>
struct transforms_type<tf2_msgs__msg__TFMessage> {
    typedef const geometry_msgs__msg__TransformStamped* type;
};

template <>
struct mutable_transforms_type<tf2_msgs__msg__TFMessage> {
    typedef geometry_msgs__msg__TransformStamped* type;
};

template <>
typename transforms_type<tf2_msgs__msg__TFMessage>::type
transforms_data(const tf2_msgs__msg__TFMessage& msg);

template <>
typename mutable_transforms_type<tf2_msgs__msg__TFMessage>::type
mutable_transforms_data(tf2_msgs__msg__TFMessage& msg);

#endif // AP_UROS_ENABLED
