#!/usr/bin/env python3
"""Read RPLIDAR S2 scans from the rplidar_s2_nucleo sketch and print them.

The NUCLEO prints one block per revolution:
    # SCAN <n> points=<p> hz=<h>
    <angle_deg>,<distance_mm>,<quality>      one line per degree with a hit
    # END

Structured like the Slamtec SDK used by rplidar_ros, so reading never waits on printing/publishing:
    RX thread       (AsyncTransceiver::_proc_rxThread)  read whatever the port has, queue the chunks
    decoder thread  (AsyncTransceiver::_proc_decoderThread + codec)  split lines, parse the blocks
    ScanHolder      (sl_lidar_driver.cpp)  two buffers: one being filled, one finished revolution;
                    swapped when a block completes, partial blocks are discarded
    main loop       (rplidar_node.cpp)  grab_scan() with a 2 s timeout -> ascend (sort by angle)
                    -> print / publish LaserScan

Usage:
    python3 lidar_reader.py                      # every scan: summary line + all values (angle:mm, 10 per row)
    python3 lidar_reader.py --values             # every scan: summary line + one value per line with quality
    python3 lidar_reader.py --summary            # only the summary line per scan
    python3 lidar_reader.py --ros                # also publish sensor_msgs/LaserScan on /scan
    python3 lidar_reader.py -p /dev/ttyACM1 -b 921600 --frame laser --topic scan

While it runs, type a board command and Enter: r = start the LiDAR again, s = stop it.

--ros needs a sourced ROS 2 install (source /opt/ros/<distro>/setup.bash), no package required.
Close the Arduino Serial Monitor first, only one program can open the port.
"""

import argparse
import math
import queue
import re
import sys
import threading
import time

import serial

SCAN_RE = re.compile(r"# SCAN (\d+) points=(\d+) hz=([\d.]+)")
POINT_RE = re.compile(r"(\d+),(\d+),(\d+)$")
BINS = 360
GRAB_TIMEOUT = 2.0              # s, the SDK's DEFAULT_TIMEOUT for grabScanDataHq


class ScanHolder:
    """Double buffer of revolutions, like the SDK's ScanHolder.

    begin() starts filling the operational buffer (the SDK does this on a sample with the SYNC bit),
    finish() hands it over as the available scan and wakes grab(), rewind() drops a partial one.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._filling = None            # (header, points) being collected, None = discard samples
        self._available = None          # last finished (header, points)

    def begin(self, header):
        with self._lock:
            self._filling = (header, [])

    def push(self, point):
        with self._lock:
            if self._filling is not None:   # samples before the first header form no scan
                self._filling[1].append(point)

    def finish(self):
        with self._lock:
            if self._filling is None:
                return
            self._available, self._filling = self._filling, None
            self._ready.set()

    def rewind(self):
        with self._lock:
            self._filling = None

    def grab(self, timeout):
        """Wait for the next finished revolution. Returns (header, points) or None on timeout."""
        if not self._ready.wait(timeout):
            return None
        with self._lock:
            self._ready.clear()
            scan, self._available = self._available, None
            return scan


class Transceiver:
    """Serial port with an RX thread and a decoder thread feeding a ScanHolder."""

    def __init__(self, port, baud, holder):
        self.port, self.baud = port, baud
        self.holder = holder
        self.status = queue.Queue()     # board messages for the main thread to print
        self._rx = queue.Queue()        # raw chunks, like the SDK's _rxQueue
        self._running = True
        self.ser = None
        self._threads = [threading.Thread(target=self._rx_thread, daemon=True),
                         threading.Thread(target=self._decoder_thread, daemon=True)]
        for t in self._threads:
            t.start()

    def send(self, data):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.write(data)
                return True
        except (serial.SerialException, OSError):
            pass
        return False

    def close(self):
        self._running = False
        self._rx.put(None)
        for t in self._threads:
            t.join(timeout=0.2)
        if self.ser is not None:
            self.ser.close()

    def _open(self):
        while self._running:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
                self.status.put(f"Reading {self.port} @ {self.baud}. Ctrl+C to stop.")
                return True
            except serial.SerialException as e:
                self.status.put(f"Cannot open {self.port}: {e} (Serial Monitor open?), retrying")
                time.sleep(0.2)
        return False

    def _rx_thread(self):
        """Only reads and queues, so bytes are drained even while the main loop is busy."""
        if not self._open():
            return
        empty_reads = 0
        while self._running:
            try:
                chunk = self.ser.read(max(1, self.ser.in_waiting))
                empty_reads = 0
            except serial.SerialException as e:
                # "readiness to read but returned no data": another program took the bytes, or the
                # board was unplugged. Only a long run of these means the port is really gone.
                empty_reads += 1
                if empty_reads < 200:
                    if empty_reads == 1 and "multiple access" in str(e):
                        self.status.put("Another program is reading this port too (close it): values get lost")
                    time.sleep(0.001)
                    continue
                self.status.put(f"Serial error: {e}, reopening")
                empty_reads = 0
                self.ser.close()
                self._rx.put(b"")       # tells the decoder the stream broke
                if not self._open():
                    return
                continue
            except OSError as e:
                self.status.put(f"Serial error: {e}, reopening")
                self.ser.close()
                self._rx.put(b"")       # tells the decoder the stream broke
                if not self._open():
                    return
                continue
            if chunk:
                self._rx.put(chunk)

    def _decoder_thread(self):
        pending = b""
        in_block = False
        skip_first = True               # opened mid-line: the first line is a fragment
        while self._running:
            chunk = self._rx.get()
            if chunk is None:
                return
            if chunk == b"":            # port reopened: a half-received block is useless
                pending, in_block, skip_first = b"", False, True
                self.holder.rewind()
                continue
            pending += chunk
            *lines, pending = pending.split(b"\n")
            if skip_first and lines:
                lines, skip_first = lines[1:], False
            for raw in lines:
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                in_block = self._decode_line(line, in_block)

    def _decode_line(self, line, in_block):
        m = SCAN_RE.match(line)
        if m:
            self.holder.begin({"scan": int(m.group(1)), "points": int(m.group(2)), "hz": float(m.group(3))})
            return True
        if line == "# END":
            if in_block:
                self.holder.finish()
            return False
        if in_block:
            m = POINT_RE.match(line)
            if m:
                self.holder.push((int(m.group(1)), int(m.group(2)), int(m.group(3))))
                return True
            self.holder.rewind()        # block cut short (restart message etc.)
        elif POINT_RE.match(line):
            return False                # values of a block whose header we missed (port opened mid-block)
        self.status.put(f"[board] {line}")  # startup info, health, "LiDAR stopped sending", ...
        return False


def ascend(points):
    """Like the SDK's ascendScanData: one entry per degree in angle order, None where there was no hit."""
    bins = [None] * BINS
    for deg, dist_mm, qual in points:
        if deg < BINS:
            bins[deg] = (dist_mm, qual)
    return bins


class RosPublisher:
    """Publishes each scan as a 360-bin LaserScan (rclpy imported only when --ros is given)."""

    def __init__(self, topic, frame_id, range_min, range_max, inverted):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan

        self.rclpy = rclpy
        self.LaserScan = LaserScan
        rclpy.init()
        self.node = rclpy.create_node("rplidar_s2_reader")
        self.pub = self.node.create_publisher(LaserScan, topic, qos_profile_sensor_data)
        self.frame_id = frame_id
        self.range_min = range_min
        self.range_max = range_max
        self.inverted = inverted
        print(f"Publishing LaserScan on /{topic.lstrip('/')} in frame '{frame_id}'")

    def publish(self, bins, hz, stamp):
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

        msg = self.LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_increment = 2.0 * math.pi / BINS
        msg.angle_max = msg.angle_increment * (BINS - 1)
        msg.scan_time = 1.0 / hz if hz > 0 else 0.0
        msg.time_increment = msg.scan_time / BINS
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = ranges
        msg.intensities = intensities
        self.pub.publish(msg)

    def now(self):
        return self.node.get_clock().now().to_msg()

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("--values", action="store_true", help="print one value per line (angle, mm, quality)")
    ap.add_argument("--summary", action="store_true", help="print only the summary line of each scan")
    ap.add_argument("--ros", action="store_true", help="publish sensor_msgs/LaserScan")
    ap.add_argument("--topic", default="scan")
    ap.add_argument("--frame", default="laser")
    ap.add_argument("--range-min", type=float, default=0.05)
    ap.add_argument("--range-max", type=float, default=30.0)
    ap.add_argument("--inverted", action="store_true", help="LiDAR mounted upside down")
    args = ap.parse_args()

    ros = None
    if args.ros:
        try:
            ros = RosPublisher(args.topic, args.frame, args.range_min, args.range_max, args.inverted)
        except ImportError as e:
            sys.exit(f"--ros needs ROS 2 sourced first ({e})")

    holder = ScanHolder()
    link = Transceiver(args.port, args.baud, holder)
    t_start = time.time()
    last_scan = t_start

    def stdin_thread():
        """Forward r / s typed in this terminal to the board."""
        for line in sys.stdin:
            cmd = line.strip().lower()[:1]
            if cmd in ("r", "s"):
                ok = link.send(cmd.encode())
                print(f"sent '{cmd}' to the board" if ok else "port not open, command not sent")
    if sys.stdin.isatty():
        threading.Thread(target=stdin_thread, daemon=True).start()

    def print_status():
        while True:
            try:
                print(link.status.get_nowait())
            except queue.Empty:
                return

    try:
        while True:
            scan = holder.grab(GRAB_TIMEOUT)
            stamp = ros.now() if ros else None      # rplidar_ros stamps at grab time too
            print_status()
            now = time.time()
            if scan is None:
                print(f"{now - t_start:7.2f}s  no scan in {now - last_scan:.0f} s")
                continue
            last_scan = now

            header, points = scan
            bins = ascend(points)
            hits = [(deg, hit[0]) for deg, hit in enumerate(bins) if hit is not None]
            nearest = min(hits, key=lambda h: h[1]) if hits else None
            near = f"nearest {nearest[1]} mm @ {nearest[0]} deg" if nearest else "no hits"
            print(f"{now - t_start:7.2f}s  scan {header['scan']:6d}  {header['hz']:5.1f} Hz  "
                  f"{len(hits):3d}/360 deg with hits  {near}")
            if args.values:
                for deg, dist in hits:
                    print(f"    {deg:3d} deg  {dist:6d} mm  q={bins[deg][1]}")
            elif not args.summary:
                for i in range(0, len(hits), 10):
                    print("    " + "  ".join(f"{deg:3d}:{dist:5d}" for deg, dist in hits[i:i + 10]))
            if ros:
                ros.publish(bins, header["hz"], stamp)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        print_status()
        if ros:
            ros.close()


if __name__ == "__main__":
    main()
