#include "AP_UROS_Type_Conversions.h"
#if AP_UROS_ENABLED

uint64_t AP_UROS_Type_Conversions::time_u64_micros(const builtin_interfaces__msg__Time& ros_time)
{
    return AP_ROS_TypeConversions::time_u64_micros(ros_time);
}

#endif // AP_UROS_ENABLED
