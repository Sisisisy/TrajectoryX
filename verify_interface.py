"""Stationary ROS check. Simulator must be running; other controllers stopped.
Creates the supplied controller with max_speed=0, verifies sensor input and
fresh zero-speed drive output, then stops it. This is NOT a lap test.
"""
import math
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from team_controller.controller_node import Controller


def main():
    rclpy.init(args=['--ros-args', '-p', 'max_speed:=0.0'])
    observer = Node('submission_interface_check')
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    controller = None
    samples = {'scan': 0, 'drive': 0, 'bad': 0}
    try:
        until = time.monotonic() + 2.0
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.1)
        if observer.count_publishers('/drive'):
            raise RuntimeError('Another /drive publisher is active. Stop teleoperation/controllers and retry.')
        if not observer.count_publishers('/scan'):
            raise RuntimeError('No /scan publisher. Start the official simulator and retry.')
        def scan_callback(msg):
            if len(msg.ranges) > 0:
                samples['scan'] += 1
        def drive_callback(msg):
            samples['drive'] += 1
            if (not math.isfinite(msg.drive.speed) or msg.drive.speed != 0.0
                    or not math.isfinite(msg.drive.steering_angle)
                    or abs(msg.drive.steering_angle) > 0.4
                    or (msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0)):
                samples['bad'] += 1
        scan_sub = observer.create_subscription(LaserScan, '/scan', scan_callback, qos_profile_sensor_data)
        drive_sub = observer.create_subscription(AckermannDriveStamped, '/drive', drive_callback, 10)
        controller = Controller()
        executor.add_node(controller)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if not samples['scan']:
            raise RuntimeError('No LiDAR received: check simulator and ROS domain / QoS.')
        if not samples['drive'] or samples['bad']:
            raise RuntimeError('Missing or invalid timestamped zero-speed drive commands.')
        if controller.last_scan is None or time.monotonic() - controller.last_scan > 0.30:
            raise RuntimeError('Controller is not receiving fresh scans.')
        if observer.count_publishers('/drive') != 1:
            raise RuntimeError('Expected exactly one /drive publisher during verification.')
        print('PASS: controller received fresh LiDAR and produced timestamped zero-speed commands.')
        print('Next: run.sh at 0.4 m/s and verify THREE timed laps with ZERO collisions in RViz.')
    finally:
        if controller is not None:
            controller.publish_drive(0.0, 0.0)
            executor.remove_node(controller)
            controller.destroy_node()
        executor.remove_node(observer)
        observer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
