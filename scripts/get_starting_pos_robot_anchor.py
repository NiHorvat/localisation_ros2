#!/usr/bin/env python3
"""
Reads the robot starting position and anchor positions from OptiTrack ROS2 topics
and outputs them in the YAML format expected by localisation_engine_node.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PoseGrabber(Node):
    """Grabs a single PoseStamped message from a given topic."""

    def __init__(self, topic: str, timeout_sec: float = 5.0):
        super().__init__('pose_grabber_' + topic.strip('/').replace('/', '_'))
        self.topic = topic
        self.timeout_sec = timeout_sec
        self.position = None
        self._sub = self.create_subscription(
            PoseStamped, topic, self._cb, 10
        )

    def _cb(self, msg: PoseStamped):
        self.position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def grab(self):
        """Spin until a message arrives or the timeout elapses."""
        start = self.get_clock().now()
        while rclpy.ok() and self.position is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > self.timeout_sec:
                break
        return self.position


def fetch_position(topic: str, timeout_sec: float = 5.0):
    node = PoseGrabber(topic, timeout_sec)
    try:
        pos = node.grab()
    finally:
        node.destroy_node()
    if pos is None:
        print(f"  [!] timed out waiting for {topic}", file=sys.stderr)
    return pos


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--anchors',
        nargs='+',
        type=int,
        default=[2, 3, 4, 5],
        help='Anchor IDs to read (default: 2 3 4 5)',
    )
    parser.add_argument(
        '--robot-topic',
        default='/optitrack/tag/pose',
        help='Topic for the robot starting pose',
    )
    parser.add_argument(
        '--anchor-topic-template',
        default='/optitrack/anchor{id}/pose',
        help='Template for anchor topics, use {id} as placeholder',
    )
    parser.add_argument(
        '--timeout', type=float, default=5.0,
        help='Seconds to wait per topic',
    )
    args = parser.parse_args()

    rclpy.init()
    try:
        # Robot
        print(f"Reading robot position from {args.robot_topic} ...", file=sys.stderr)
        robot = fetch_position(args.robot_topic, args.timeout)

        # Anchors
        anchor_positions = []
        for aid in args.anchors:
            topic = args.anchor_topic_template.format(id=aid)
            print(f"Reading anchor {aid} from {topic} ...", file=sys.stderr)
            pos = fetch_position(topic, args.timeout)
            if pos is not None:
                anchor_positions.extend(pos)
    finally:
        rclpy.shutdown()

    # Output the two YAML lines
    print()
    if robot is not None:
        print(f"    robot_starting_position: [{robot[0]},{robot[1]},{robot[2]}]")
    else:
        print("    robot_starting_position: [<missing>]")

    anchor_comment = "    # " + ", ".join(f"anchor_{a}" for a in args.anchors)
    print(anchor_comment)
    flat = ", ".join(repr(v) for v in anchor_positions)
    print(f"    anchor_positions_param: [{flat}]")


if __name__ == '__main__':
    main()