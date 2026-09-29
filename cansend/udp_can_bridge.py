#!/usr/bin/env python3
"""
UDP (Ethernet) <-> virtual SocketCAN bridge for the NUCLEO-F207ZG sketch can_udp/can_udp.ino.

    PC can0 (vcan)  <--->  this script  <--- UDP / Ethernet --->  STM32  <--->  CAN bus (1 Mbit/s)

  can0 -> STM32 : every frame sent on can0 (e.g. `cansend can0 001#FFFFFFFFFFFFFFFC`)
                  is sent to the board, which puts it on the real bus.
  STM32 -> can0 : every frame the board receives from the real bus is injected
                  into can0, so `candump can0` shows it.
  Status text from the board (CANSTAT / CANALERT / ERROR) is printed here.

The virtual interface is created automatically (uses sudo if not run as root):
    modprobe vcan
    ip link add dev can0 type vcan
    ip link set up can0

Run:
    python3 udp_can_bridge.py                                  # board 192.168.200.177
    python3 udp_can_bridge.py --board 192.168.200.177 --iface can0
    python3 udp_can_bridge.py --delete-on-exit                 # remove can0 on Ctrl+C

Test in other terminals:
    candump can0
    cansend can0 001#FFFFFFFFFFFFFFFC

Datagram format (must match can_udp.ino):
    'C' 'B' <type> <count>, then
      type 1 (frames): count x 16-byte struct can_frame {u32 can_id; u8 len; u8 pad[3]; u8 data[8]}
      type 2 (text)  : ASCII status line
      type 3 (hello) : keep-alive from the PC, no payload
"""

import argparse
import os
import socket
import struct
import subprocess
import sys
import threading
import time

BOARD_IP = "192.168.200.177"
BOARD_PORT = 5700   # board listens here
PC_PORT = 5701      # this script listens here

MAGIC = b"CB"
PKT_FRAMES = 1
PKT_TEXT = 2
PKT_HELLO = 3
HEADER_FMT = "<2sBB"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
MAX_FRAMES_PER_PKT = 32
HELLO_INTERVAL = 1.0
BOARD_SILENT_WARN = 3.0

# struct can_frame { u32 can_id; u8 len; u8 pad[3]; u8 data[8]; }  (same on the wire)
CAN_FRAME_FMT = "<IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

print_lock = threading.Lock()


def log(msg):
    with print_lock:
        print(msg, flush=True)


def frame_str(can_id, data):
    if can_id & CAN_EFF_FLAG:
        text = f"{can_id & CAN_EFF_MASK:08X}"
    else:
        text = f"{can_id & CAN_SFF_MASK:03X}"
    if can_id & CAN_RTR_FLAG:
        return text + "#R"
    return f"{text}#{data.hex().upper()}"


# ─── Virtual CAN interface ───────────────────────────────────────────────────

def run_root(cmd):
    if os.geteuid() != 0:
        cmd = ["sudo"] + cmd
    log("$ " + " ".join(cmd))
    subprocess.run(cmd, check=True)


def iface_exists(iface):
    return os.path.exists(f"/sys/class/net/{iface}")


def iface_is_up(iface):
    with open(f"/sys/class/net/{iface}/flags") as f:
        return int(f.read(), 16) & 0x1  # IFF_UP


def setup_vcan(iface):
    """Create and bring up a virtual CAN interface. Returns True if it was created."""
    created = False
    try:
        if not iface_exists(iface):
            run_root(["modprobe", "vcan"])
            run_root(["ip", "link", "add", "dev", iface, "type", "vcan"])
            created = True
        if not iface_is_up(iface):
            run_root(["ip", "link", "set", "up", iface])
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        log(f"ERROR: could not create virtual CAN interface '{iface}': {e}")
        sys.exit(1)
    log(f"Virtual CAN interface {iface} is up" + (" (created)" if created else ""))
    return created


def delete_vcan(iface):
    try:
        run_root(["ip", "link", "delete", iface])
    except subprocess.CalledProcessError as e:
        log(f"WARNING: could not delete {iface}: {e}")


def open_can(iface):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((iface,))
    # A raw socket does not receive frames it sent itself (CAN_RAW_RECV_OWN_MSGS
    # defaults to 0), so frames injected from the board are never echoed back to it.
    return sock


# ─── Bridge threads ──────────────────────────────────────────────────────────

class Bridge:
    def __init__(self, can_sock, udp_sock, board, iface, verbose):
        self.can = can_sock
        self.udp = udp_sock
        self.board = board
        self.iface = iface
        self.verbose = verbose
        self.stop = threading.Event()
        self.last_board_rx = 0.0
        self.last_canstat = None
        self.tx_count = 0
        self.rx_count = 0

    def send_hello(self):
        """Keep-alive so the board unicasts to this PC."""
        pkt = struct.pack(HEADER_FMT, MAGIC, PKT_HELLO, 0)
        warned = False
        while not self.stop.is_set():
            try:
                self.udp.sendto(pkt, self.board)
            except OSError as e:
                log(f"WARNING: cannot reach board {self.board[0]}: {e}")

            silent = time.monotonic() - self.last_board_rx
            if silent > BOARD_SILENT_WARN and not warned:
                log(f"WARNING: nothing from board {self.board[0]}:{self.board[1]} "
                    f"for {BOARD_SILENT_WARN:.0f}s (cable / IP / sketch?)")
                warned = True
            elif silent <= BOARD_SILENT_WARN:
                warned = False
            self.stop.wait(HELLO_INTERVAL)

    def can_to_udp(self):
        """can0 -> board. Frames that arrive together go in one datagram."""
        self.can.settimeout(0.2)
        while not self.stop.is_set():
            try:
                raw = self.can.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            except OSError as e:
                log(f"ERROR: read from {self.iface} failed: {e}")
                self.stop.set()
                break

            frames = [raw]
            self.can.setblocking(False)
            try:
                while len(frames) < MAX_FRAMES_PER_PKT:
                    frames.append(self.can.recv(CAN_FRAME_SIZE))
            except (BlockingIOError, socket.timeout):
                pass
            finally:
                self.can.settimeout(0.2)

            payload = b""
            for f in frames:
                can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, f[:CAN_FRAME_SIZE])
                if can_id & CAN_ERR_FLAG:
                    continue
                payload += struct.pack(CAN_FRAME_FMT, can_id, min(dlc, 8), data)
                if self.verbose:
                    log(f"{self.iface} -> BUS: {frame_str(can_id, data[:min(dlc, 8)])}")

            count = len(payload) // CAN_FRAME_SIZE
            if count == 0:
                continue
            try:
                self.udp.sendto(struct.pack(HEADER_FMT, MAGIC, PKT_FRAMES, count) + payload,
                                self.board)
                self.tx_count += count
            except OSError as e:
                log(f"ERROR: UDP send to board failed: {e}")

    def udp_to_can(self):
        """board -> can0 (frames) / console (status text)."""
        self.udp.settimeout(0.2)
        while not self.stop.is_set():
            try:
                pkt, addr = self.udp.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError as e:
                log(f"ERROR: UDP receive failed: {e}")
                self.stop.set()
                break

            if len(pkt) < HEADER_SIZE:
                continue
            magic, ptype, count = struct.unpack_from(HEADER_FMT, pkt)
            if magic != MAGIC:
                continue
            self.last_board_rx = time.monotonic()

            if ptype == PKT_TEXT:
                self.handle_text(pkt[HEADER_SIZE:].decode("ascii", errors="replace").strip())
            elif ptype == PKT_FRAMES:
                self.handle_frames(pkt, count)

    def handle_text(self, text):
        if text.startswith("CANSTAT"):
            # Heartbeat every second: only print when something changed.
            if text == self.last_canstat:
                return
            self.last_canstat = text
        log(f"STM32 >> {text}")

    def handle_frames(self, pkt, count):
        for i in range(count):
            off = HEADER_SIZE + i * CAN_FRAME_SIZE
            if off + CAN_FRAME_SIZE > len(pkt):
                break
            can_id, dlc, data = struct.unpack_from(CAN_FRAME_FMT, pkt, off)
            dlc = min(dlc, 8)
            try:
                # Native struct can_frame for the kernel
                self.can.send(struct.pack("=IB3x8s", can_id, dlc, data))
                self.rx_count += 1
            except OSError as e:
                log(f"ERROR: write to {self.iface} failed: {e}")
                continue
            if self.verbose:
                log(f"BUS -> {self.iface}: {frame_str(can_id, data[:dlc])}")

    def run(self):
        threads = [
            threading.Thread(target=self.send_hello, daemon=True),
            threading.Thread(target=self.can_to_udp, daemon=True),
            threading.Thread(target=self.udp_to_can, daemon=True),
        ]
        for t in threads:
            t.start()
        try:
            while not self.stop.is_set():
                self.stop.wait(0.5)
        except KeyboardInterrupt:
            log("\nStopping bridge...")
        finally:
            self.stop.set()
            for t in threads:
                t.join(timeout=1)
            log(f"Frames {self.iface} -> bus: {self.tx_count}   bus -> {self.iface}: {self.rx_count}")


def main():
    ap = argparse.ArgumentParser(description="Bridge the STM32 UDP CAN sketch to a virtual can0")
    ap.add_argument("--board", default=BOARD_IP, help=f"board IP (default {BOARD_IP})")
    ap.add_argument("--board-port", type=int, default=BOARD_PORT, help="board UDP port")
    ap.add_argument("--port", type=int, default=PC_PORT, help="local UDP port to listen on")
    ap.add_argument("--iface", default="can0", help="virtual CAN interface name")
    ap.add_argument("--delete-on-exit", action="store_true",
                    help="delete the virtual interface when the bridge stops")
    ap.add_argument("-q", "--quiet", action="store_true", help="do not print every frame")
    args = ap.parse_args()

    created = setup_vcan(args.iface)
    can_sock = open_can(args.iface)

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        udp_sock.bind(("0.0.0.0", args.port))
    except OSError as e:
        log(f"ERROR: cannot listen on UDP port {args.port}: {e}")
        sys.exit(1)

    board = (args.board, args.board_port)
    log(f"Bridge running: {args.iface}  <->  UDP {board[0]}:{board[1]} (listening on {args.port})")
    log(f"Try:  candump {args.iface}    |    cansend {args.iface} 001#FFFFFFFFFFFFFFFC")
    log("Ctrl+C to stop.\n")

    bridge = Bridge(can_sock, udp_sock, board, args.iface, verbose=not args.quiet)
    try:
        bridge.run()
    finally:
        udp_sock.close()
        can_sock.close()
        if args.delete_on_exit and created:
            delete_vcan(args.iface)
        log("Closed.")


if __name__ == "__main__":
    main()
