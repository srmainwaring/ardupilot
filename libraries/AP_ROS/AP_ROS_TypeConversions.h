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
