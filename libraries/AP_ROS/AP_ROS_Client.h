#pragma once

#include <stdint.h>
#include <string.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Math/AP_Math.h>

#include <AP_RTC/AP_RTC.h>

#include "AP_ROS_Frames.h"
#include "AP_ROS_TypeConversions.h"


class AP_ROS_Client
{
public:

    template <typename BatteryState>
    static void update_battery_state(BatteryState& msg);

    template <typename Clock>
    static void update_clock(Clock& msg);

    template <typename GeoPoseStamped>
    static void update_geopose_stamped(GeoPoseStamped& msg);

    template <typename PoseStamped>
    static void update_pose_stamped(PoseStamped& msg);

    template <typename TwistStamped>
    static void update_twist_stamped(TwistStamped& msg);

    template <typename TFMessage>
    static void update_static_transforms(TFMessage& msg);

    template <typename Time>
    static void update_time(Time& msg);

};

template <typename BatteryState>
void AP_ROS_Client::update_battery_state(BatteryState& msg)
{
    const uint8_t instance = 0;

    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES) {
        return;
    }

    update_time(msg.header.stamp);
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

    if (battery.has_cell_voltages(instance)) {
        const uint16_t* cellVoltages = battery.get_cell_voltages(instance).cells;
        std::copy(cellVoltages, cellVoltages + AP_BATT_MONITOR_CELLS_MAX, msg.cell_voltage.data);
    }
}

template <typename Clock>
void AP_ROS_Client::update_clock(Clock& msg)
{
    update_time(msg.clock);
}

template <typename GeoPoseStamped>
void AP_ROS_Client::update_geopose_stamped(GeoPoseStamped& msg)
{
    update_time(msg.header.stamp);
    strcpy(mutable_string_data(msg.header.frame_id), BASE_LINK_FRAME_ID);

    auto &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    Location loc;
    if (ahrs.get_location(loc)) {
        msg.pose.position.latitude = loc.lat * 1E-7;
        msg.pose.position.longitude = loc.lng * 1E-7;
        msg.pose.position.altitude = loc.alt * 0.01; // Transform from cm to m
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
        Quaternion transformation(sqrtF(2) * 0.5, 0, 0, sqrtF(2) * 0.5); // Z axis 90 degree rotation
        orientation = aux * transformation;
        msg.pose.orientation.w = orientation[0];
        msg.pose.orientation.x = orientation[1];
        msg.pose.orientation.y = orientation[2];
        msg.pose.orientation.z = orientation[3];
    }
}

template <typename PoseStamped>
void AP_ROS_Client::update_pose_stamped(PoseStamped& msg)
{
    update_time(msg.header.stamp);
    strcpy(mutable_string_data(msg.header.frame_id), BASE_LINK_FRAME_ID);

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

template <typename TwistStamped>
void AP_ROS_Client::update_twist_stamped(TwistStamped& msg)
{
    update_time(msg.header.stamp);
    strcpy(mutable_string_data(msg.header.frame_id), BASE_LINK_FRAME_ID);

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

template <typename TFMessage>
void AP_ROS_Client::update_static_transforms(TFMessage& msg)
{
    transforms_mutable_size<TFMessage>(msg) = 0;

    auto &gps = AP::gps();
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        const auto gps_type = gps.get_type(i);
        if (gps_type == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
            continue;
        }
        auto* transform_stamped_seq = transforms_mutable_data<TFMessage>(msg);
        auto& transform_stamped = transform_stamped_seq[i];

        // update_time(transforms_data(msg)[i].header.stamp);
        // update_time(transform_stamped.header.stamp);
        char gps_frame_id[16];
        //! @todo should GPS frame ID's be 0 or 1 indexed in ROS?
        hal.util->snprintf(gps_frame_id, sizeof(gps_frame_id), "GPS_%u", i);
        // strcpy(string_data(transforms_data(msg)[i].header.frame_id), BASE_LINK_FRAME_ID);
        // strcpy(string_data(transforms_data(msg)[i].child_frame_id), gps_frame_id);
        // The body-frame offsets
        // X - Forward
        // Y - Right
        // Z - Down
        // https://ardupilot.org/copter/docs/common-sensor-offset-compensation.html#sensor-position-offset-compensation

        // const auto offset = gps.get_antenna_offset(i);

        // In ROS REP 103, it follows this convention
        // X - Forward
        // Y - Left
        // Z - Up
        // https://www.ros.org/reps/rep-0103.html#axis-orientation

        // transforms_data(msg)[i].transform.translation.x = offset[0];
        // transforms_data(msg)[i].transform.translation.y = -1 * offset[1];
        // transforms_data(msg)[i].transform.translation.z = -1 * offset[2];

        transforms_size(msg)++;
    }
}

template <typename Time>
void AP_ROS_Client::update_time(Time& msg)
{
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        utc_usec = AP_HAL::micros64();
    }
    msg.sec = utc_usec / 1000000ULL;
    msg.nanosec = (utc_usec % 1000000ULL) * 1000UL;
}

