#!/usr/bin/env python3
"""ROS 2 node for the buttions.ino sketch: button + 2 relays on a NUCLEO-F207ZG over Ethernet (UDP).

Single file, no other module or package needed. Talks to the NUCLEO by its IP address:
    - binds local_port and sends 'h' to board_ip:board_port every second, so the board sends its
      status to this PC (it broadcasts until some PC says hello)
    - status datagrams ("BR" v1, 20 bytes, 5 Hz and at once on every change) are published on
        <prefix>/relay1        std_msgs/Bool     relay 1 ON
        <prefix>/relay2        std_msgs/Bool     relay 2 ON
        <prefix>/button        std_msgs/Bool     button pressed now
        <prefix>/button_event  std_msgs/String   "pressed", "short_press", "long_press", "boot",
                                                 "reboot_requested" (only when it happens)
        <prefix>/status        diagnostic_msgs/DiagnosticStatus  everything above + uptime,
                               reset cause, last hold time; level STALE when the board is silent
    - text datagrams (startup, reboot) go to the ROS log
    - service <prefix>/reboot  std_srvs/Trigger: soft reboot of the STM32 (NVIC_SystemReset).
      Returns success once the board has come back up (boot status with reset cause "software"),
      or failure if it does not answer / come back within reboot_timeout.
      The relays are OFF after the reboot.

Parameters (defaults in brackets):
    board_ip        [192.168.200.177]
    board_port      [5600]
    local_port      [5601]
    topic_prefix    [stm32]
    stale_timeout   [1.0]   s without status -> /status level STALE
    reboot_timeout  [15.0]  s to wait for the board to come back after REBOOT

Usage:
    source /opt/ros/<distro>/setup.bash
    python3 button_relay_node.py
    ros2 topic echo /stm32/relay1
    ros2 topic echo /stm32/button_event
    ros2 topic echo /stm32/status
    ros2 service call /stm32/reboot std_srvs/srv/Trigger
"""

import socket
import struct
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

STATUS_PACKET = struct.Struct("<2sBBIIBBBBI")   # magic, version, type, seq, uptime, flags, event, reset, res, hold
STATUS_MAGIC = b"BR"
STATUS_TYPE = 1
HELLO_S = 1.0                                   # board forgets the PC after 5 s without a packet
ACK_TIMEOUT_S = 1.0                             # board answers REBOOT at once

FLAG_RELAY1 = 0x01
FLAG_RELAY2 = 0x02
FLAG_BUTTON = 0x04
FLAG_LONG_ARMED = 0x08

EV_NONE, EV_BOOT, EV_PRESSED, EV_SHORT, EV_LONG, EV_REBOOT = range(6)
EVENT_NAMES = {EV_BOOT: "boot", EV_PRESSED: "pressed", EV_SHORT: "short_press",
               EV_LONG: "long_press", EV_REBOOT: "reboot_requested"}
RESET_CAUSES = {0: "unknown", 1: "power-on", 2: "reset pin", 3: "software (reboot)",
                4: "independent watchdog", 5: "window watchdog", 6: "brown-out", 7: "low-power"}
RESET_SOFTWARE = 3


class ButtonRelayNode(Node):
    def __init__(self):
        super().__init__("button_relay_node")
        self.board_ip = self.declare_parameter("board_ip", "192.168.200.177").value
        self.board_port = self.declare_parameter("board_port", 5600).value
        local_port = self.declare_parameter("local_port", 5601).value
        prefix = self.declare_parameter("topic_prefix", "stm32").value.strip("/")
        self.stale_timeout = float(self.declare_parameter("stale_timeout", 1.0).value)
        self.reboot_timeout = float(self.declare_parameter("reboot_timeout", 15.0).value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", local_port))
        self.sock.settimeout(0.2)

        self.relay1_pub = self.create_publisher(Bool, f"{prefix}/relay1", 10)
        self.relay2_pub = self.create_publisher(Bool, f"{prefix}/relay2", 10)
        self.button_pub = self.create_publisher(Bool, f"{prefix}/button", 10)
        self.event_pub = self.create_publisher(String, f"{prefix}/button_event", 10)
        self.status_pub = self.create_publisher(DiagnosticStatus, f"{prefix}/status", 10)

        # Own group: the reboot call waits for the board without blocking the stale-check timer
        self.create_service(Trigger, f"{prefix}/reboot", self._on_reboot,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.5, self._check_stale)

        self.last_status_time = 0.0
        self.last_seq = None
        self.stale_reported = False
        self._lock = threading.Lock()
        self._reboot_ack = threading.Event()
        self._reboot_boot = threading.Event()
        self._boot_cause = None

        self._running = True
        self._worker = threading.Thread(target=self._recv_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(f"Listening on UDP {local_port}, board {self.board_ip}:{self.board_port}; "
                               f"topics /{prefix}/{{relay1,relay2,button,button_event,status}}, "
                               f"service /{prefix}/reboot")

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
        while self._running and rclpy.ok():
            now = time.time()
            if now - last_hello >= HELLO_S:
                self._send(b"h")
                last_hello = now
            try:
                data, (ip, _) = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    time.sleep(0.1)     # e.g. ICMP port unreachable while the board boots
                continue
            if ip != self.board_ip:
                self.get_logger().warn(f"ignoring UDP from {ip} (board_ip is {self.board_ip})",
                                       throttle_duration_sec=10.0)
                continue

            if data.startswith(STATUS_MAGIC) and len(data) == STATUS_PACKET.size:
                self._on_status(data)
            else:
                self.get_logger().info(f"[board] {data.decode('ascii', errors='replace').strip()}")

    def _on_status(self, data):
        _, version, ptype, seq, uptime_ms, flags, event, reset_cause, _, hold_ms = STATUS_PACKET.unpack(data)
        if version != 1 or ptype != STATUS_TYPE:
            self.get_logger().warn(f"unknown status packet version {version} type {ptype}",
                                   throttle_duration_sec=5.0)
            return

        with self._lock:
            if self.stale_reported:
                self.get_logger().info(f"board {self.board_ip} online again")
                self.stale_reported = False
            self.last_status_time = time.time()
            self.last_seq = seq

        relay1 = bool(flags & FLAG_RELAY1)
        relay2 = bool(flags & FLAG_RELAY2)
        button = bool(flags & FLAG_BUTTON)
        cause = RESET_CAUSES.get(reset_cause, str(reset_cause))

        self.relay1_pub.publish(Bool(data=relay1))
        self.relay2_pub.publish(Bool(data=relay2))
        self.button_pub.publish(Bool(data=button))

        if event in EVENT_NAMES:
            name = EVENT_NAMES[event]
            self.event_pub.publish(String(data=name))
            if event == EV_BOOT:
                self.get_logger().info(f"board started, reset cause: {cause}")
                self._boot_cause = reset_cause
                self._reboot_boot.set()
            elif event == EV_REBOOT:
                self._reboot_ack.set()
            elif event in (EV_SHORT, EV_LONG):
                self.get_logger().info(f"{name} ({hold_ms} ms): relay1 {'ON' if relay1 else 'OFF'}, "
                                       f"relay2 {'ON' if relay2 else 'OFF'}")

        msg = DiagnosticStatus()
        msg.level = DiagnosticStatus.OK
        msg.name = "stm32 button/relay"
        msg.message = f"relay1 {'ON' if relay1 else 'OFF'}, relay2 {'ON' if relay2 else 'OFF'}"
        msg.hardware_id = self.board_ip
        msg.values = [
            KeyValue(key="relay1", value=str(relay1)),
            KeyValue(key="relay2", value=str(relay2)),
            KeyValue(key="button_pressed", value=str(button)),
            KeyValue(key="long_press_armed", value=str(bool(flags & FLAG_LONG_ARMED))),
            KeyValue(key="event", value=EVENT_NAMES.get(event, "none")),
            KeyValue(key="last_hold_ms", value=str(hold_ms)),
            KeyValue(key="uptime_s", value=f"{uptime_ms / 1000.0:.1f}"),
            KeyValue(key="reset_cause", value=cause),
            KeyValue(key="seq", value=str(seq)),
        ]
        self.status_pub.publish(msg)

    def _check_stale(self):
        with self._lock:
            silent = time.time() - self.last_status_time
            if silent < self.stale_timeout:
                return
            first = not self.stale_reported
            self.stale_reported = True
        if first:
            self.get_logger().warn(f"no status from {self.board_ip} for {self.stale_timeout:.1f} s")
        msg = DiagnosticStatus()
        msg.level = DiagnosticStatus.STALE
        msg.name = "stm32 button/relay"
        msg.message = "no status from the board"
        msg.hardware_id = self.board_ip
        self.status_pub.publish(msg)

    # ---------------------------------------------------------------- reboot service

    def _on_reboot(self, _request, response):
        self._reboot_ack.clear()
        self._reboot_boot.clear()
        self._boot_cause = None
        self.get_logger().info(f"soft reboot of {self.board_ip} requested")

        if not self._send(b"REBOOT"):
            response.success = False
            response.message = f"cannot send to {self.board_ip}:{self.board_port}"
            return response
        if not self._reboot_ack.wait(ACK_TIMEOUT_S):
            response.success = False
            response.message = f"board {self.board_ip} did not answer the reboot request"
            self.get_logger().error(response.message)
            return response

        start = time.time()
        if not self._reboot_boot.wait(self.reboot_timeout):
            response.success = False
            response.message = (f"board accepted the reboot but did not come back within "
                                f"{self.reboot_timeout:.0f} s (Ethernet link?)")
            self.get_logger().error(response.message)
            return response

        cause = RESET_CAUSES.get(self._boot_cause, str(self._boot_cause))
        response.success = self._boot_cause == RESET_SOFTWARE
        response.message = f"board rebooted in {time.time() - start:.1f} s, reset cause: {cause}"
        (self.get_logger().info if response.success else self.get_logger().warn)(response.message)
        return response

    def destroy_node(self):
        self._running = False
        self._worker.join(timeout=0.5)
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ButtonRelayNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
