#!/usr/bin/env python3
"""ROS 2 node for the RPLIDAR S2 on the rplidar_s2_nucleo sketch.

Publishes sensor_msgs/LaserScan (360 bins, one per degree) and offers the same motor services as
rplidar_ros. The serial side (RX thread, decoder thread, ScanHolder) is reused from lidar_reader.py,
so this file has to stay in the same folder.

Parameters (defaults in brackets):
    serial_port   [/dev/ttyACM0]
    serial_baudrate [921600]
    topic_name    [scan]
    frame_id      [laser]
    range_min     [0.05]  m
    range_max     [30.0]  m
    inverted      [false] LiDAR mounted upside down

Services (std_srvs/srv/Empty):
    start_motor   sends 'r' to the board
    stop_motor    sends 's' to the board

Usage:
    source /opt/ros/<distro>/setup.bash
    python3 lidar_node.py
    python3 lidar_node.py --ros-args -p serial_port:=/dev/ttyACM1 -p frame_id:=laser_link
    ros2 service call /stop_motor std_srvs/srv/Empty

Close the Arduino Serial Monitor first, only one program can open the port.
"""

import math
import queue
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from lidar_reader import BINS, GRAB_TIMEOUT, ScanHolder, Transceiver, ascend


class RplidarS2Node(Node):
    def __init__(self):
        super().__init__("rplidar_s2_node")
        self.port = self.declare_parameter("serial_port", "/dev/ttyACM0").value
        self.baud = self.declare_parameter("serial_baudrate", 921600).value
        topic = self.declare_parameter("topic_name", "scan").value
        self.frame_id = self.declare_parameter("frame_id", "laser").value
        self.range_min = float(self.declare_parameter("range_min", 0.05).value)
        self.range_max = float(self.declare_parameter("range_max", 30.0).value)
        self.inverted = self.declare_parameter("inverted", False).value

        self.pub = self.create_publisher(LaserScan, topic, qos_profile_sensor_data)
        self.create_service(Empty, "start_motor", lambda req, res: self._send_cmd(b"r", res))
        self.create_service(Empty, "stop_motor", lambda req, res: self._send_cmd(b"s", res))

        self.holder = ScanHolder()
        self.link = Transceiver(self.port, self.baud, self.holder)
        self._running = True
        self._worker = threading.Thread(target=self._grab_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(f"Publishing LaserScan on /{topic.lstrip('/')} in frame '{self.frame_id}'")

    def _send_cmd(self, cmd, res):
        if self.link.send(cmd):
            self.get_logger().info(f"sent '{cmd.decode()}' to the board")
        else:
            self.get_logger().warn("port not open, command not sent")
        return res

    def _log_status(self):
        while True:
            try:
                self.get_logger().info(self.link.status.get_nowait())
            except queue.Empty:
                return

    def _grab_loop(self):
        """Like rplidar_node.cpp's publish loop: grab, stamp, ascend, publish."""
        last_scan = time.time()
        while self._running and rclpy.ok():
            scan = self.holder.grab(GRAB_TIMEOUT)
            stamp = self.get_clock().now().to_msg()     # rplidar_ros stamps at grab time too
            self._log_status()
            if scan is None:
                self.get_logger().warn(f"no scan in {time.time() - last_scan:.0f} s",
                                       throttle_duration_sec=5.0)
                continue
            last_scan = time.time()
            header, points = scan
            self._publish(ascend(points), header["hz"], stamp)

    def _publish(self, bins, hz, stamp):
        ranges = [math.inf] * BINS
        intensities = [0.0] * BINS
        for deg, hit in enumerate(bins):
            if hit is None:
                continue
            r = hit[0] / 1000.0
            if not (self.range_min <= r <= self.range_max):
                continue
            # RPLIDAR angles run clockwise, ROS angles counter-clockwise
            i = deg if self.inverted else (BINS - deg) % BINS
            ranges[i] = r
            intensities[i] = float(hit[1])

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_increment = 2.0 * math.pi / BINS
        msg.angle_max = msg.angle_increment * (BINS - 1)
        msg.scan_time = 1.0 / hz if hz > 0 else 0.0
        msg.time_increment = msg.scan_time / BINS
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = intensities
        self.pub.publish(msg)

    def destroy_node(self):
        self._running = False
        self._worker.join(timeout=GRAB_TIMEOUT + 0.5)
        self.link.close()
        self._log_status()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RplidarS2Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
