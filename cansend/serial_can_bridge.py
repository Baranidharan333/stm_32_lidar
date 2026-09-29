#!/usr/bin/env python3
"""
Serial <-> SocketCAN bridge for the NUCLEO-F207ZG CAN sketch (can/can.ino).

    PC SocketCAN (can0)  <--->  this script  <--- USB serial --->  STM32  <--->  CAN bus (500 kbit/s)

  can0 -> STM32 : every frame sent on can0 (e.g. `cansend can0 123#11223344`)
                  is written to the STM32 as "123#11223344\n"; the STM32 puts it on the bus.
  STM32 -> can0 : every "RX ID=0x.. DLC=.. DATA=.." line the STM32 prints
                  (a frame received from the real bus) is injected into can0,
                  so `candump can0` shows it.

One-time setup of the virtual interface (needs root):
    sudo modprobe vcan
    sudo ip link add dev can0 type vcan
    sudo ip link set up can0

Run:
    python3 serial_can_bridge.py                     # /dev/ttyACM0, can0
    python3 serial_can_bridge.py --port /dev/ttyACM1 --iface can0

Test in other terminals:
    candump can0
    cansend can0 123#11223344
"""

import argparse
import re
import socket
import struct
import sys
import threading
import time

import serial

# struct can_frame { u32 can_id; u8 len; u8 pad[3]; u8 data[8]; }
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

# Matches the line printed by readCANMessages() in can.ino, e.g.
#   RX ID=0x123 DLC=4 DATA=11 22 33 44 STD
#   RX ID=0x7FF DLC=0 DATA= STD RTR
RX_LINE_RE = re.compile(
    r"^RX ID=0x(?P<id>[0-9A-Fa-f]+)\s+DLC=(?P<dlc>\d+)\s+DATA=(?P<data>[0-9A-Fa-f ]*?)\s*"
    r"(?P<kind>STD|EXT)(?P<rtr>\s+RTR)?\s*$"
)

print_lock = threading.Lock()

# Flow control: the STM32 has a 64-byte serial RX buffer and blocks while it
# prints each "TX ..." reply, so commands sent back-to-back overflow it and get
# corrupted. Send one command, then wait for its reply before sending the next.
ACK_TIMEOUT = 0.5
stm32_ack = threading.Event()


def is_ack_line(line):
    """True for the STM32's reply to a command: 'TX ... TX queued' or any ERROR."""
    return line.startswith("TX ID=") or line.startswith("ERROR")


def log(msg):
    with print_lock:
        print(msg, flush=True)


def pack_frame(can_id, data, extended=False, remote=False):
    if extended:
        can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG
    else:
        can_id &= CAN_SFF_MASK
    if remote:
        can_id |= CAN_RTR_FLAG
    return struct.pack(CAN_FRAME_FMT, can_id, len(data), data.ljust(8, b"\x00"))


def unpack_frame(raw):
    can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw[:CAN_FRAME_SIZE])
    return can_id, data[:min(dlc, 8)]


def parse_rx_line(line):
    """Return (id, data, extended, remote) for an STM32 'RX ...' line, else None."""
    m = RX_LINE_RE.match(line)
    if not m:
        return None
    data = bytes.fromhex(m.group("data").replace(" ", ""))
    dlc = int(m.group("dlc"))
    if dlc > 8 or (not m.group("rtr") and len(data) != dlc):
        return None
    return int(m.group("id"), 16), data, m.group("kind") == "EXT", bool(m.group("rtr"))


def open_can(iface):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((iface,))
    except OSError as e:
        log(f"ERROR: cannot open CAN interface '{iface}': {e}")
        log("Create it first:")
        log("    sudo modprobe vcan")
        log(f"    sudo ip link add dev {iface} type vcan")
        log(f"    sudo ip link set up {iface}")
        sys.exit(1)
    # A raw socket does not receive frames it sent itself (CAN_RAW_RECV_OWN_MSGS
    # defaults to 0), so frames we inject from the STM32 are never echoed back to it.
    return sock


def serial_to_can(ser, sock, iface, stop):
    """STM32 serial output -> can0."""
    while not stop.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException as e:
            log(f"Serial error: {e}")
            stop.set()
            break
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue

        if is_ack_line(line):
            stm32_ack.set()

        frame = parse_rx_line(line)
        if frame is None:
            # Banner, "TX ... TX queued" acknowledgements, ERROR messages, ...
            log(f"STM32 >> {line}")
            continue

        can_id, data, extended, remote = frame
        try:
            sock.send(pack_frame(can_id, data, extended, remote))
        except OSError as e:
            log(f"ERROR: write to {iface} failed: {e}")
            continue
        log(f"BUS -> {iface}: {can_id:03X}#{data.hex().upper()}"
            f"{' EXT' if extended else ''}{' RTR' if remote else ''}")


def can_to_serial(ser, sock, iface, stop):
    """can0 -> STM32 ("ID#DATA\\n")."""
    sock.settimeout(0.2)
    while not stop.is_set():
        try:
            raw = sock.recv(CAN_FRAME_SIZE)
        except socket.timeout:
            continue
        except OSError as e:
            log(f"ERROR: read from {iface} failed: {e}")
            stop.set()
            break

        can_id, data = unpack_frame(raw)
        if can_id & CAN_ERR_FLAG:
            continue
        if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG):
            # can.ino only transmits standard 11-bit data frames.
            log(f"SKIP {iface} frame {can_id & CAN_EFF_MASK:X}: "
                "extended/RTR frames not supported by the STM32 sketch")
            continue

        cmd = f"{can_id & CAN_SFF_MASK:03X}#{data.hex().upper()}"
        stm32_ack.clear()
        try:
            ser.write((cmd + "\n").encode("ascii"))
        except serial.SerialException as e:
            log(f"Serial error: {e}")
            stop.set()
            break
        log(f"{iface} -> BUS: {cmd}")

        # Frames arriving on can0 meanwhile wait in the socket's receive queue.
        if not stm32_ack.wait(ACK_TIMEOUT):
            log(f"WARNING: no reply from STM32 for {cmd} within {ACK_TIMEOUT}s")


def main():
    ap = argparse.ArgumentParser(description="Bridge STM32 serial CAN sketch to SocketCAN")
    ap.add_argument("--port", default="/dev/ttyACM0", help="STM32 serial port")
    ap.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    ap.add_argument("--iface", default="can0", help="SocketCAN interface (vcan)")
    args = ap.parse_args()

    sock = open_can(args.iface)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=2)
    except serial.SerialException as e:
        log(f"ERROR: cannot open {args.port}: {e}")
        log("If it is a permission problem: sudo usermod -aG dialout $USER  (then log out/in)")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    log(f"Bridge running: {args.port} @ {args.baud}  <->  {args.iface}")
    log(f"Try:  candump {args.iface}    |    cansend {args.iface} 123#11223344")
    log("Ctrl+C to stop.\n")

    stop = threading.Event()
    threads = [
        threading.Thread(target=serial_to_can, args=(ser, sock, args.iface, stop), daemon=True),
        threading.Thread(target=can_to_serial, args=(ser, sock, args.iface, stop), daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        while not stop.is_set():
            stop.wait(0.5)
    except KeyboardInterrupt:
        log("\nStopping bridge...")
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=1)
        ser.close()
        sock.close()
        log("Closed.")


if __name__ == "__main__":
    main()
