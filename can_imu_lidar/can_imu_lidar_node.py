#!/usr/bin/env python3
"""ROS 2 node for the can_imu_lidar sketch: RPLIDAR S2 + BNO085 IMU + CAN bus over Ethernet (UDP).

Single file, no other module needed. Talks to the NUCLEO by its IP address:
    - binds local_port and sends 'h' to board_ip:board_port every second, so the board sends its
      data to this PC (it broadcasts until some PC says hello)
    - scan datagrams  ("RPS2", 1096 bytes, 10 Hz)   -> sensor_msgs/LaserScan  on /scan
    - IMU datagrams   (0xAA55 v3, 56 bytes, 100 Hz) -> sensor_msgs/Imu        on /imu/data
    - CAN datagrams   ('CB' type 1, SocketCAN frames) <-> virtual CAN interface can0
                      (created automatically: modprobe vcan; ip link add can0 type vcan; ip link set up can0)
    - CAN status      ('CB' type 2: CANSTAT / CANALERT / ERROR) -> ROS log (no topic)
    - text datagrams (startup, health, "LiDAR stopped sending", ...) go to the ROS log

So `candump can0` shows what the motors send and `cansend can0 001#FFFFFFFFFFFFFFFC` goes to the
real bus. Any SocketCAN program (python-can, damiao tools, ...) can use can0.

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
    can_iface     [can0]  virtual CAN interface
    create_vcan   [true]  create can_iface if missing (as root, or with passwordless sudo)
    log_can_frames [false] log every CAN frame in both directions

If the node is not root and sudo asks for a password, create can0 once by hand:
    sudo modprobe vcan && sudo ip link add dev can0 type vcan && sudo ip link set up can0
The node keeps retrying and picks it up.

Usage:
    source /opt/ros/<distro>/setup.bash
    python3 can_imu_lidar_node.py
    python3 can_imu_lidar_node.py --ros-args -p board_ip:=192.168.200.177 -p laser_frame:=laser_link
    ros2 topic hz /scan
    ros2 topic hz /imu/data
    candump can0
"""

import math
import os
import socket
import struct
import subprocess
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

CAN_MAGIC = b"CB"
CAN_PKT_FRAMES = 1
CAN_PKT_TEXT = 2
CAN_HEADER = struct.Struct("<2sBB")             # magic, type, count
CAN_WIRE_FRAME = struct.Struct("<IB3x8s")       # can_id, len, pad, data (little-endian on the wire)
CAN_KERNEL_FRAME = struct.Struct("=IB3x8s")     # struct can_frame for the kernel (native order)
CAN_MAX_FRAMES = 32                             # per datagram, as in the sketch
CAN_RETRY_S = 2.0
CANSTAT_FAULT_FIELDS = ("STATE", "TEC", "REC", "TXFAIL", "TXDROP")

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF


class CanImuLidarNode(Node):
    def __init__(self):
        super().__init__("can_imu_lidar_node")
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
        self.can_iface = self.declare_parameter("can_iface", "can0").value
        self.create_vcan = self.declare_parameter("create_vcan", True).value
        self.log_can_frames = self.declare_parameter("log_can_frames", False).value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(("", local_port))
        self.sock.settimeout(0.2)

        self.scan_pub = self.create_publisher(LaserScan, scan_topic, qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, imu_topic, qos_profile_sensor_data)

        self.can_sock = None                    # set by the CAN thread once can0 is open
        self.last_canstat = None
        self.can_to_bus = 0
        self.can_from_bus = 0

        self._running = True
        self._worker = threading.Thread(target=self._recv_loop, daemon=True)
        self._can_worker = threading.Thread(target=self._can_loop, daemon=True)
        self._worker.start()
        self._can_worker.start()
        self.get_logger().info(f"Listening on UDP {local_port}, board {self.board_ip}:{self.board_port}; "
                               f"/{scan_topic.lstrip('/')} ({self.laser_frame}), "
                               f"/{imu_topic.lstrip('/')} ({self.imu_frame}), "
                               f"CAN <-> {self.can_iface}")

    # ---------------------------------------------------------------- UDP to the board

    def _send(self, data):
        try:
            self.sock.sendto(data, (self.board_ip, self.board_port))
            return True
        except OSError as e:
            self.get_logger().warn(f"cannot send to {self.board_ip}:{self.board_port}: {e}",
                                   throttle_duration_sec=5.0)
            return False

    # ---------------------------------------------------------------- receiving from the board

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
            elif data.startswith(CAN_MAGIC) and len(data) >= CAN_HEADER.size:
                self._on_can(data)
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

    # ---------------------------------------------------------------- CAN: board -> can0

    def _on_can(self, data):
        _, ptype, count = CAN_HEADER.unpack_from(data)

        if ptype == CAN_PKT_TEXT:
            text = data[CAN_HEADER.size:].decode("ascii", errors="replace").strip()
            if text.startswith("CANSTAT"):
                # Heartbeat every second: log only when a fault field changes (not TXOK / RX counting up)
                fields = dict(kv.split("=", 1) for kv in text.split()[1:] if "=" in kv)
                key = tuple(fields.get(k) for k in CANSTAT_FAULT_FIELDS)
                if key != self.last_canstat:
                    healthy = (fields.get("STATE") == "RUNNING" and fields.get("TEC") == "0"
                               and fields.get("REC") == "0")
                    if self.last_canstat is not None or not healthy:
                        log = self.get_logger().info if healthy else self.get_logger().warn
                        log(f"[can] {text}")
                    self.last_canstat = key
            elif text.startswith("ERROR"):
                self.get_logger().warn(f"[can] {text}")
            else:
                self.get_logger().info(f"[can] {text}")
            return

        if ptype != CAN_PKT_FRAMES:
            return
        can_sock = self.can_sock
        for i in range(count):
            off = CAN_HEADER.size + i * CAN_WIRE_FRAME.size
            if off + CAN_WIRE_FRAME.size > len(data):
                break
            can_id, dlc, payload = CAN_WIRE_FRAME.unpack_from(data, off)
            dlc = min(dlc, 8)
            if self.log_can_frames:
                self.get_logger().info(f"[can] bus -> {self.can_iface}: {_frame_str(can_id, payload[:dlc])}")
            if can_sock is None:
                self.get_logger().warn(f"CAN frame from the bus dropped: {self.can_iface} not open",
                                       throttle_duration_sec=5.0)
                continue
            try:
                can_sock.send(CAN_KERNEL_FRAME.pack(can_id, dlc, payload))
                self.can_from_bus += 1
            except OSError as e:
                self.get_logger().warn(f"write to {self.can_iface} failed: {e}", throttle_duration_sec=5.0)

    # ---------------------------------------------------------------- CAN: can0 -> board

    def _setup_vcan(self):
        """Create and bring up the virtual CAN interface. True if it exists and is up."""
        sys_path = f"/sys/class/net/{self.can_iface}"
        cmds = []
        if not os.path.exists(sys_path):
            cmds = [["modprobe", "vcan"],
                    ["ip", "link", "add", "dev", self.can_iface, "type", "vcan"],
                    ["ip", "link", "set", "up", self.can_iface]]
        else:
            with open(f"{sys_path}/flags") as f:
                if not int(f.read(), 16) & 0x1:           # IFF_UP
                    cmds = [["ip", "link", "set", "up", self.can_iface]]
        if not cmds:
            return True
        if not self.create_vcan:
            return False

        prefix = [] if os.geteuid() == 0 else ["sudo", "-n"]  # -n: never wait for a password
        for cmd in cmds:
            result = subprocess.run(prefix + cmd, capture_output=True, text=True)
            if result.returncode != 0:
                self.get_logger().error(
                    f"cannot create {self.can_iface} ({' '.join(prefix + cmd)}: "
                    f"{result.stderr.strip() or result.returncode}). Create it once by hand:\n"
                    f"    sudo modprobe vcan && sudo ip link add dev {self.can_iface} type vcan "
                    f"&& sudo ip link set up {self.can_iface}\n"
                    f"  retrying every {CAN_RETRY_S:.0f} s")
                return False
        self.get_logger().info(f"created virtual CAN interface {self.can_iface}")
        return True

    def _open_can(self):
        tried_create = False
        while self._running and rclpy.ok():
            if not tried_create or os.path.exists(f"/sys/class/net/{self.can_iface}"):
                ok = self._setup_vcan()
                tried_create = True
                if ok:
                    try:
                        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                        s.bind((self.can_iface,))
                        s.settimeout(0.2)
                        # A raw socket does not get its own frames back (CAN_RAW_RECV_OWN_MSGS = 0),
                        # so frames injected from the bus are never sent back to the board.
                        self.get_logger().info(f"CAN bridge up: {self.can_iface} <-> board")
                        return s
                    except OSError as e:
                        self.get_logger().error(f"cannot open {self.can_iface}: {e}",
                                                throttle_duration_sec=10.0)
            time.sleep(CAN_RETRY_S)
        return None

    def _can_loop(self):
        self.can_sock = self._open_can()
        s = self.can_sock
        while self._running and rclpy.ok() and s is not None:
            try:
                raw = s.recv(CAN_KERNEL_FRAME.size)
            except socket.timeout:
                continue
            except OSError as e:
                self.get_logger().error(f"read from {self.can_iface} failed: {e}; reopening")
                self.can_sock = None
                s.close()
                self.can_sock = s = self._open_can()
                continue

            # Frames that arrive together go to the board in one datagram
            frames = [raw]
            s.setblocking(False)
            try:
                while len(frames) < CAN_MAX_FRAMES:
                    frames.append(s.recv(CAN_KERNEL_FRAME.size))
            except (BlockingIOError, InterruptedError):
                pass
            finally:
                s.settimeout(0.2)

            payload = bytearray()
            for f in frames:
                can_id, dlc, data = CAN_KERNEL_FRAME.unpack(f[:CAN_KERNEL_FRAME.size])
                if can_id & CAN_ERR_FLAG:
                    continue
                dlc = min(dlc, 8)
                payload += CAN_WIRE_FRAME.pack(can_id, dlc, data)
                if self.log_can_frames:
                    self.get_logger().info(f"[can] {self.can_iface} -> bus: {_frame_str(can_id, data[:dlc])}")
            count = len(payload) // CAN_WIRE_FRAME.size
            if count and self._send(CAN_HEADER.pack(CAN_MAGIC, CAN_PKT_FRAMES, count) + payload):
                self.can_to_bus += count

    def destroy_node(self):
        self._running = False
        self._worker.join(timeout=0.5)
        self._can_worker.join(timeout=0.5)
        self.sock.close()
        if self.can_sock is not None:
            self.can_sock.close()
        super().destroy_node()


def _diag(v):
    return [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]


def _frame_str(can_id, data):
    text = f"{can_id & CAN_EFF_MASK:08X}" if can_id & CAN_EFF_FLAG else f"{can_id & CAN_SFF_MASK:03X}"
    if can_id & CAN_RTR_FLAG:
        return text + "#R"
    return f"{text}#{data.hex().upper()}"


def main(args=None):
    rclpy.init(args=args)
    node = CanImuLidarNode()
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
