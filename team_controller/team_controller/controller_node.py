"""ROS 2 Jazzy interface for the reactive controller."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

from .algorithm import Driver


class Controller(Node):
    def __init__(self):
        super().__init__('team_controller')
        self.declare_parameter('max_speed', 0.8)
        configured = float(self.get_parameter('max_speed').value)
        if not math.isfinite(configured) or configured < 0:
            raise ValueError('max_speed must be finite and nonnegative')
        self.max_speed = min(0.8, configured)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.last_scan = None
        self.command = (0.0, 0.0)
        self.driver = Driver(self.max_speed)
        self.pose = None
        self.pose_time = None
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.odom_subscription = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.on_odom, 10)
        self.timer = self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f'Controller ready; maximum speed {self.max_speed:.2f} m/s. '
            'Stop teleoperation. Simulator testing is required.')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_time = time.monotonic()

    def on_scan(self, scan):
        self.command = (0.0, 0.0)
        now = time.monotonic()
        pose = None
        if self.pose is not None and now - self.pose_time < 0.5:
            pose = self.pose  # stale odometry -> purely reactive driving
        try:
            self.command = self.driver.step(
                scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max, now, pose)
            self.last_scan = now
        except (ValueError, TypeError, OverflowError, ZeroDivisionError):
            self.last_scan = None

    def publish_drive(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.publisher.publish(msg)

    def on_timer(self):
        if self.last_scan is None or time.monotonic() - self.last_scan > 0.30:
            self.publish_drive(0.0, 0.0)
        else:
            self.publish_drive(*self.command)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_drive(0.0, 0.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
