# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
Combine pose and twist subscriptions and publish odometry.

Usage

ros2 run ardupilot_dds_tests odometry_publisher --ros-args
    -r pose:=ap/pose/filtered
    -r twist:=ap/twist/filtered
    -r odometry:=ap/odometry/filtered
    -p use_sim_time:=true
    -p frame_id:=iris/odom
    -p child_frame_id:=iris
"""
import message_filters
import rclpy
import rclpy.node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy


class OdometryPublisher(rclpy.node.Node):
    """
    Combine pose and twist subscriptions and publish odometry.

    Subscribed topics
    `pose` (geometry_msgs/PoseStamped)
    `twist` (geometry_msgs/TwistStamped)

    Published topics
    `odometry` (nav_msgs/Odometry)

    Parameters
    `frame_id` (string)
        The odometry frame, default "odom".
    `child_frame_id` (string)
        The robot base frame, default "base_link".
    """

    def __init__(self) -> None:
        """Initialise the node."""
        super().__init__("odometry_publisher")

        # Declare and acquire parameters.
        self.declare_parameter("frame_id", "odom")
        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        self.declare_parameter("child_frame_id", "base_link")
        self._child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )

        # Declare topics.
        self._pose_topic = "pose"
        self._twist_topic = "twist"
        self._odom_topic = "odometry"

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.publish_odometry)

    def start_subscribers(self) -> None:
        """Start the subscribers."""
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pose_sub = message_filters.Subscriber(
            self, PoseStamped, self._pose_topic, qos_profile=qos_profile
        )
        self._twist_sub = message_filters.Subscriber(
            self, TwistStamped, self._twist_topic, qos_profile=qos_profile
        )

        self._ts = message_filters.TimeSynchronizer(
            [self._pose_sub, self._twist_sub], 10
        )
        self._ts.registerCallback(self.filter_callback)

        self._last_clock = Clock()
        self._last_pose = PoseStamped()
        self._last_twist = TwistStamped()

    def filter_callback(self, pose, twist):
        """Process `PoseStamped` and `TwistStamped` using message filter."""
        # self.get_logger().info("{}\n{}\n".format(pose, twist))
        self._last_pose = pose
        self._last_twist = twist

    def publish_odometry(self):
        """Publish `Odometry` to topic `/odometry`."""
        # Time - will use /clock if param use_sim_time:=true.
        now = self.get_clock().now()

        # No information for pose covariance.
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = self._last_pose.pose

        # No information for twist covariance.
        twist_with_cov = TwistWithCovariance()
        twist_with_cov.twist = self._last_twist.twist

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame_id
        odom.pose = pose_with_cov
        odom.twist = twist_with_cov

        self._odom_pub.publish(odom)


def main(args=None) -> None:
    """Entrypoint for the odometry_publisher node."""
    rclpy.init(args=args)
    node = OdometryPublisher()
    node.start_subscribers()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
