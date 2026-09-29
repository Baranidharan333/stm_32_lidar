#!/usr/bin/env python3
"""ROS 2 node for the RPLIDAR S2 on the rplidar_s2_nucleo_udp sketch, over Ethernet (UDP).

Single file, no other module from this folder needed. Talks to the NUCLEO by its IP address:
    - binds local_port and sends 'h' to board_ip:board_port every second, so the board sends its
      scans to this PC (it broadcasts them until some PC says hello)
    - each scan datagram (1096 bytes, see the sketch) becomes one sensor_msgs/LaserScan
    - text datagrams from the board (startup, health, "LiDAR stopped sending") go to the ROS log

Parameters (defaults in brackets):
    board_ip      [192.168.200.177]
    board_port    [5600]
    local_port    [5601]
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
    python3 lidar_udp_node.py
    python3 lidar_udp_node.py --ros-args -p board_ip:=192.168.200.177 -p frame_id:=laser_link
    ros2 service call /stop_motor std_srvs/srv/Empty
"""

import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

BINS = 360
MAGIC = b"RPS2"
HEADER = struct.Struct("<4sBBHIHH")             # magic, version, reserved, points, block, hz10, hits
PACKET_SIZE = HEADER.size + BINS * 2 + BINS     # 1096
HELLO_S = 1.0                                   # board forgets the PC after 5 s without a packet
NO_SCAN_S = 2.0                                 # the SDK's DEFAULT_TIMEOUT for grabScanDataHq


class RplidarS2UdpNode(Node):
    def __init__(self):
        super().__init__("rplidar_s2_udp_node")
        self.board_ip = self.declare_parameter("board_ip", "192.168.200.177").value
        self.board_port = self.declare_parameter("board_port", 5600).value
        local_port = self.declare_parameter("local_port", 5601).value
        topic = self.declare_parameter("topic_name", "scan").value
        self.frame_id = self.declare_parameter("frame_id", "laser").value
        self.range_min = float(self.declare_parameter("range_min", 0.05).value)
        self.range_max = float(self.declare_parameter("range_max", 30.0).value)
        self.inverted = self.declare_parameter("inverted", False).value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(("", local_port))
        self.sock.settimeout(0.2)

        self.pub = self.create_publisher(LaserScan, topic, qos_profile_sensor_data)
        self.create_service(Empty, "start_motor", lambda req, res: self._send_cmd(b"r", res))
        self.create_service(Empty, "stop_motor", lambda req, res: self._send_cmd(b"s", res))

        self._running = True
        self._worker = threading.Thread(target=self._recv_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(f"Listening on UDP {local_port}, board {self.board_ip}:{self.board_port}; "
                               f"publishing LaserScan on /{topic.lstrip('/')} in frame '{self.frame_id}'")

    def _send(self, data):
        try:
            self.sock.sendto(data, (self.board_ip, self.board_port))
            return True
        except OSError as e:
            self.get_logger().warn(f"cannot send to {self.board_ip}:{self.board_port}: {e}",
                                   throttle_duration_sec=5.0)
            return False

    def _send_cmd(self, cmd, res):
        if self._send(cmd):
            self.get_logger().info(f"sent '{cmd.decode()}' to the board")
        return res

    def _recv_loop(self):
        last_hello = 0.0
        last_scan = time.time()
        last_block = None
        while self._running and rclpy.ok():
            now = time.time()
            if now - last_hello >= HELLO_S:
                self._send(b"h")
                last_hello = now
            if now - last_scan >= NO_SCAN_S:
                self.get_logger().warn(f"no scan from {self.board_ip} in {now - last_scan:.0f} s",
                                       throttle_duration_sec=5.0)
            try:
                data, (ip, _) = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    time.sleep(0.1)     # e.g. ICMP port unreachable while the board boots
                continue
            stamp = self.get_clock().now().to_msg()     # rplidar_ros stamps at grab time too
            if ip != self.board_ip:
                self.get_logger().warn(f"ignoring UDP from {ip} (board_ip is {self.board_ip})",
                                       throttle_duration_sec=10.0)
                continue

            if not data.startswith(MAGIC):
                self.get_logger().info(f"[board] {data.decode('ascii', errors='replace').strip()}")
                continue
            if len(data) != PACKET_SIZE:
                self.get_logger().warn(f"scan datagram of {len(data)} bytes, expected {PACKET_SIZE}",
                                       throttle_duration_sec=5.0)
                continue
            _, version, _, points, block, hz10, hits = HEADER.unpack_from(data)
            if version != 1:
                self.get_logger().warn(f"unknown packet version {version}", throttle_duration_sec=5.0)
                continue
            if last_block is not None and block > last_block + 1:
                self.get_logger().warn(f"lost {block - last_block - 1} scan(s)", throttle_duration_sec=5.0)
            last_block = block
            last_scan = now
            dist = struct.unpack_from(f"<{BINS}H", data, HEADER.size)
            qual = data[HEADER.size + BINS * 2:]
            self._publish(dist, qual, hz10 / 10.0, stamp)

    def _publish(self, dist, qual, hz, stamp):
        ranges = [math.inf] * BINS
        intensities = [0.0] * BINS
        for deg in range(BINS):
            if dist[deg] == 0:
                continue
            r = dist[deg] / 1000.0
            if not (self.range_min <= r <= self.range_max):
                continue
            # RPLIDAR angles run clockwise, ROS angles counter-clockwise
            i = deg if self.inverted else (BINS - deg) % BINS
            ranges[i] = r
            intensities[i] = float(qual[deg])

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
        self._worker.join(timeout=0.5)
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RplidarS2UdpNode()
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
