#!/usr/bin/env python3
"""
Graphs the Euclidean distance between:
    /optitrack/tag/pose   (geometry_msgs/PoseStamped)
    /tag/marker           (visualization_msgs/Marker)

Live plot of distance vs. time. On exit (Ctrl-C or window close), prints
mean / median / stddev / max and optionally saves the data to CSV.

Usage:
    python3 graph_marker_distance.py
    python3 graph_marker_distance.py --csv distances.csv --duration 30
"""

import argparse
import csv
import math
import signal
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class DistanceNode(Node):
    def __init__(self, pose_topic: str, marker_topic: str, max_points: int):
        super().__init__('distance_grapher')

        # Latest known positions (x, y, z)
        self.pose_pos = None
        self.marker_pos = None

        # Stamped samples: (t_seconds_since_start, distance)
        self.samples = deque(maxlen=max_points)
        self.lock = threading.Lock()
        self.t0 = time.monotonic()

        self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 50)
        self.create_subscription(Marker, marker_topic, self._marker_cb, 50)

        self.get_logger().info(f"Subscribed to {pose_topic} and {marker_topic}")

    def _pose_cb(self, msg: PoseStamped):
        self.pose_pos = (msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z)
        self._record()

    def _marker_cb(self, msg: Marker):
        self.marker_pos = (msg.pose.position.x,
                           msg.pose.position.y,
                           msg.pose.position.z)
        self._record()

    def _record(self):
        if self.pose_pos is None or self.marker_pos is None:
            return
        dx = self.pose_pos[0] - self.marker_pos[0]
        dy = self.pose_pos[1] - self.marker_pos[1]
        dz = self.pose_pos[2] - self.marker_pos[2]
        d = math.sqrt(dx * dx + dy * dy + dz * dz)
        t = time.monotonic() - self.t0
        with self.lock:
            self.samples.append((t, d))

    def snapshot(self):
        with self.lock:
            return list(self.samples)


def run_ros_thread(executor, node):
    try:
        executor.spin()
    except Exception as e:
        node.get_logger().error(f"executor stopped: {e}")


def print_stats(samples, csv_path=None):
    if not samples:
        print("\nNo paired samples were received.", file=sys.stderr)
        return
    dists = [d for _, d in samples]
    n = len(dists)
    mean = sum(dists) / n
    var = sum((d - mean) ** 2 for d in dists) / n
    stddev = math.sqrt(var)
    sorted_d = sorted(dists)
    median = sorted_d[n // 2] if n % 2 else 0.5 * (sorted_d[n // 2 - 1] + sorted_d[n // 2])
    print(f"\nSamples: {n}")
    print(f"  mean   = {mean:.4f} m")
    print(f"  median = {median:.4f} m")
    print(f"  stddev = {stddev:.4f} m")
    print(f"  min    = {min(dists):.4f} m")
    print(f"  max    = {max(dists):.4f} m")

    if csv_path:
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_seconds', 'distance_m'])
            for t, d in samples:
                w.writerow([f"{t:.6f}", f"{d:.6f}"])
        print(f"  saved {n} rows to {csv_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pose-topic', default='/optitrack/tag/pose')
    parser.add_argument('--marker-topic', default='/tag/marker')
    parser.add_argument('--max-points', type=int, default=5000,
                        help='Ring-buffer size for the live plot')
    parser.add_argument('--interval', type=float, default=0.1,
                        help='Plot refresh interval (s)')
    parser.add_argument('--duration', type=float, default=None,
                        help='Stop after N seconds (default: run until closed)')
    parser.add_argument('--csv', default=None,
                        help='Optional path to save samples on exit')
    args = parser.parse_args()

    rclpy.init()
    node = DistanceNode(args.pose_topic, args.marker_topic, args.max_points)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=run_ros_thread,
                                  args=(executor, node), daemon=True)
    ros_thread.start()

    # Let Ctrl-C interrupt matplotlib's blocking calls.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], linewidth=1.4)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('distance [m]')
    ax.set_title(f"|{args.pose_topic} - {args.marker_topic}|")
    ax.grid(True, alpha=0.3)

    start = time.monotonic()
    try:
        while plt.fignum_exists(fig.number):
            data = node.snapshot()
            if data:
                ts = [t for t, _ in data]
                ds = [d for _, d in data]
                line.set_data(ts, ds)
                ax.relim()
                ax.autoscale_view()
                # Live current-value label
                ax.set_title(
                    f"|{args.pose_topic} - {args.marker_topic}|   "
                    f"current = {ds[-1]:.3f} m   n = {len(ds)}"
                )
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(args.interval)

            if args.duration is not None and (time.monotonic() - start) >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print_stats(node.snapshot(), args.csv)
        # Keep the window up briefly if it's still open
        if plt.fignum_exists(fig.number):
            plt.ioff()
            plt.show()


if __name__ == '__main__':
    main()