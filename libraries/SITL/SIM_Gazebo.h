/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connection for ardupilot version of Gazebo
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_GAZEBO_ENABLED
#define HAL_SIM_GAZEBO_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if HAL_SIM_GAZEBO_ENABLED

#include "SIM_Aircraft.h"
#include <AP_HAL/utility/Socket.h>

namespace SITL {

/*
  Gazebo simulator
 */
class Gazebo : public Aircraft {
public:
    Gazebo(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Gazebo(frame_str);
    }

    /*  Create and set in/out socket for Gazebo simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:
    /*
      packet sent to Gazebo
     */
    struct servo_packet {
      // size matches sitl_input upstream
      float motor_speed[16];
    };

    /*
      reply packet sent from Gazebo to ArduPilot
     */
    struct fdm_packet {
      // Enum to set bitmask. See: SIM_JSON.h
      enum DataKey {
        TIMESTAMP   = 1U << 0,  // timestamp
        GYRO        = 1U << 1,  // imu_angular_velocity_rpy
        ACCEL_BODY  = 1U << 2,  // imu_linear_acceleration_xyz
        POSITION    = 1U << 3,  // position_xyz
        EULER_ATT   = 1U << 4,  // not used
        QUAT_ATT    = 1U << 5,  // imu_orientation_quat
        VELOCITY    = 1U << 6,  // velocity_xyz
        RNG_1       = 1U << 7,  // not used
        RNG_2       = 1U << 8,  // not used
        RNG_3       = 1U << 9,  // not used
        RNG_4       = 1U << 10, // not used
        RNG_5       = 1U << 11, // not used
        RNG_6       = 1U << 12, // not used
        WIND_DIR    = 1U << 13, // wind_apparent.direction
        WIND_SPD    = 1U << 14, // wind_apparent.speed
      };
      uint16_t bitmask;

      double timestamp;  // in seconds
      double imu_angular_velocity_rpy[3];
      double imu_linear_acceleration_xyz[3];
      double imu_orientation_quat[4];
      double velocity_xyz[3];
      double position_xyz[3];
      struct {
        float direction;  // apparent wind direction in radians
        float speed;      // apparent wind speed in m/s 
      } wind_apparent;
    };

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);
    void drain_sockets();

    double last_timestamp;

    SocketAPM socket_sitl;
    const char *_gazebo_address = "127.0.0.1";
    int _gazebo_port = 9002;
    static const uint64_t GAZEBO_TIMEOUT_US = 5000000;
};

}  // namespace SITL


#endif  // HAL_SIM_GAZEBO_ENABLED
