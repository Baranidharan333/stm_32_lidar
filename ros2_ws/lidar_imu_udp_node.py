#!/usr/bin/env python3
"""ROS 2 node for the lidar_imu_nucleo_udp sketch: RPLIDAR S2 + BNO085 IMU over Ethernet (UDP).

Single file, no other module from this folder needed. Talks to the NUCLEO by its IP address:
    - binds local_port and sends 'h' to board_ip:board_port every second, so the board sends its
      data to this PC (it broadcasts until some PC says hello)
    - scan datagrams  ("RPS2", 1096 bytes, 10 Hz)  -> sensor_msgs/LaserScan   on /scan
    - IMU datagrams   (0xAA55 v3, 56 bytes, 100 Hz) -> sensor_msgs/Imu         on /imu/data
    - text datagrams (startup, health, "LiDAR stopped sending", ...) go to the ROS log

Parameters (defaults in brackets):
    board_ip      [192.168.200.177]
    board_port    [5600]
    local_port    [5601]
    scan_topic    [scan]
    imu_topic     [imu/data]
    laser_frame   [laser]
    imu_frame     [imu_link]
    range_min     [0.05]  m
    range_max     [30.0]  m
    inverted      [false] LiDAR mounted upside down
    orientation_variance       [0.01]   rad^2, diagonal of orientation_covariance
    angular_velocity_variance  [0.0004] (rad/s)^2
    linear_acceleration_variance [0.01] (m/s^2)^2

Usage:
    source /opt/ros/<distro>/setup.bash
    python3 lidar_imu_udp_node.py
    python3 lidar_imu_udp_node.py --ros-args -p board_ip:=192.168.200.177 -p laser_frame:=laser_link
    ros2 topic hz /scan
    ros2 topic hz /imu/data
"""

import math
import socket
import struct
import threading
import time
import zlib

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

BINS = 360
SCAN_MAGIC = b"RPS2"
SCAN_HEADER = struct.Struct("<4sBBHIHH")        # magic, version, reserved, points, block, hz10, hits
SCAN_SIZE = SCAN_HEADER.size + BINS * 2 + BINS  # 1096
IMU_SYNC = b"\xaa\x55"
IMU_PACKET = struct.Struct("<2sBBII10fI")       # sync, version, reserved, seq, time_us, q wxyz, gyro, accel, crc
HELLO_S = 1.0                                   # board forgets the PC after 5 s without a packet
NO_SCAN_S = 2.0                                 # the SDK's DEFAULT_TIMEOUT for grabScanDataHq
NO_IMU_S = 1.0


class LidarImuUdpNode(Node):
    def __init__(self):
        super().__init__("lidar_imu_udp_node")
        self.board_ip = self.declare_parameter("board_ip", "192.168.200.177").value
        self.board_port = self.declare_parameter("board_port", 5600).value
        local_port = self.declare_parameter("local_port", 5601).value
        scan_topic = self.declare_parameter("scan_topic", "scan").value
        imu_topic = self.declare_parameter("imu_topic", "imu/data").value
        self.laser_frame = self.declare_parameter("laser_frame", "laser").value
        self.imu_frame = self.declare_parameter("imu_frame", "imu_link").value
        self.range_min = float(self.declare_parameter("range_min", 0.05).value)
        self.range_max = float(self.declare_parameter("range_max", 30.0).value)
        self.inverted = self.declare_parameter("inverted", False).value
        self.orientation_var = float(self.declare_parameter("orientation_variance", 0.01).value)
        self.gyro_var = float(self.declare_parameter("angular_velocity_variance", 0.0004).value)
        self.accel_var = float(self.declare_parameter("linear_acceleration_variance", 0.01).value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(("", local_port))
        self.sock.settimeout(0.2)

        self.scan_pub = self.create_publisher(LaserScan, scan_topic, qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, imu_topic, qos_profile_sensor_data)

        self._running = True
        self._worker = threading.Thread(target=self._recv_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(f"Listening on UDP {local_port}, board {self.board_ip}:{self.board_port}; "
                               f"/{scan_topic.lstrip('/')} ({self.laser_frame}), "
                               f"/{imu_topic.lstrip('/')} ({self.imu_frame})")

    # ---------------------------------------------------------------- hello to the board

    def _send(self, data):
        try:
            self.sock.sendto(data, (self.board_ip, self.board_port))
            return True
        except OSError as e:
            self.get_logger().warn(f"cannot send to {self.board_ip}:{self.board_port}: {e}",
                                   throttle_duration_sec=5.0)
            return False

    # ---------------------------------------------------------------- receiving

    def _recv_loop(self):
        last_hello = 0.0
        self.last_scan = self.last_imu = time.time()
        self.last_block = self.last_imu_seq = None
        while self._running and rclpy.ok():
            now = time.time()
            if now - last_hello >= HELLO_S:
                self._send(b"h")
                last_hello = now
            if now - self.last_scan >= NO_SCAN_S:
                self.get_logger().warn(f"no scan from {self.board_ip} in {now - self.last_scan:.0f} s",
                                       throttle_duration_sec=5.0)
            if now - self.last_imu >= NO_IMU_S:
                self.get_logger().warn(f"no IMU data from {self.board_ip} in {now - self.last_imu:.0f} s",
                                       throttle_duration_sec=5.0)
            try:
                data, (ip, _) = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    time.sleep(0.1)     # e.g. ICMP port unreachable while the board boots
                continue
            stamp = self.get_clock().now().to_msg()     # stamped at receive time, like rplidar_ros
            if ip != self.board_ip:
                self.get_logger().warn(f"ignoring UDP from {ip} (board_ip is {self.board_ip})",
                                       throttle_duration_sec=10.0)
                continue

            if data.startswith(SCAN_MAGIC):
                self._on_scan(data, stamp)
            elif data.startswith(IMU_SYNC) and len(data) == IMU_PACKET.size:
                self._on_imu(data, stamp)
            else:
                self.get_logger().info(f"[board] {data.decode('ascii', errors='replace').strip()}")

    def _on_scan(self, data, stamp):
        if len(data) != SCAN_SIZE:
            self.get_logger().warn(f"scan datagram of {len(data)} bytes, expected {SCAN_SIZE}",
                                   throttle_duration_sec=5.0)
            return
        _, version, _, points, block, hz10, hits = SCAN_HEADER.unpack_from(data)
        if version != 1:
            self.get_logger().warn(f"unknown scan packet version {version}", throttle_duration_sec=5.0)
            return
        if self.last_block is not None and block > self.last_block + 1:
            self.get_logger().warn(f"lost {block - self.last_block - 1} scan(s)", throttle_duration_sec=5.0)
        self.last_block = block
        self.last_scan = time.time()
        dist = struct.unpack_from(f"<{BINS}H", data, SCAN_HEADER.size)
        qual = data[SCAN_HEADER.size + BINS * 2:]
        hz = hz10 / 10.0

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
        msg.header.frame_id = self.laser_frame
        msg.angle_min = 0.0
        msg.angle_increment = 2.0 * math.pi / BINS
        msg.angle_max = msg.angle_increment * (BINS - 1)
        msg.scan_time = 1.0 / hz if hz > 0 else 0.0
        msg.time_increment = msg.scan_time / BINS
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = intensities
        self.scan_pub.publish(msg)

    def _on_imu(self, data, stamp):
        fields = IMU_PACKET.unpack(data)
        _, version, _, seq, _device_us = fields[:5]
        qw, qx, qy, qz, gx, gy, gz, ax, ay, az = fields[5:15]
        crc = fields[15]
        if zlib.crc32(data[:-4]) & 0xFFFFFFFF != crc:
            self.get_logger().warn("IMU packet with bad CRC dropped", throttle_duration_sec=5.0)
            return
        if version != 3:
            self.get_logger().warn(f"unknown IMU packet version {version}", throttle_duration_sec=5.0)
            return
        if self.last_imu_seq is not None and seq > self.last_imu_seq + 1:
            self.get_logger().warn(f"lost {seq - self.last_imu_seq - 1} IMU packet(s)", throttle_duration_sec=5.0)
        self.last_imu_seq = seq
        self.last_imu = time.time()

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        msg.orientation.w = float(qw)
        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        msg.linear_acceleration.x = float(ax)     # includes gravity, as sensor_msgs/Imu expects
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        msg.orientation_covariance = _diag(self.orientation_var)
        msg.angular_velocity_covariance = _diag(self.gyro_var)
        msg.linear_acceleration_covariance = _diag(self.accel_var)
        self.imu_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        self._worker.join(timeout=0.5)
        self.sock.close()
        super().destroy_node()


def _diag(v):
    return [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]


def main(args=None):
    rclpy.init(args=args)
    node = LidarImuUdpNode()
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
