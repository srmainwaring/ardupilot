// Class for handling type conversions for ROS.

#pragma once

#include <stdint.h>

class AP_ROS_TypeConversions
{
public:

    // Convert ROS time to a uint64_t [μS]
    template <typename TTime>
    static uint64_t time_u64_micros(const TTime& ros_time);
};

template <typename TTime>
uint64_t AP_ROS_TypeConversions::time_u64_micros(const TTime& ros_time)
{
    return (uint64_t(ros_time.sec) * 1000000ULL) + (ros_time.nanosec / 1000ULL);
}

// string accessor templates
template <typename S>
const char* string_data(const S* str);

template <typename S>
const char* string_data(const S& str);

// sequence accessor templates
// see: https://stackoverflow.com/questions/15911890/overriding-return-type-in-function-template-specialization

template <typename T>
uint32_t transforms_size(const T& msg);

template <typename T>
struct transforms_type{ typedef T* type; };

template <typename T>
typename transforms_type<T>::type transforms_data(const T& msg);

