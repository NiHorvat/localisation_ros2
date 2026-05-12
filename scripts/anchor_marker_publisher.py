#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
 
ANCHORS = [
    [1.0, 0.0, 0.0],
    [0.0, 2.0, 0.0],
    [0.0, 0.0, 3.0],
]
 
class AnchorPublisher(Node):
    def __init__(self):
        super().__init__('anchor_publisher')
        self.pub = self.create_publisher(MarkerArray, '/anchors/marker', 10)
        self.create_timer(1.0, self.publish)
 
    def publish(self):
        markers = MarkerArray()
        for i, (x, y, z) in enumerate(ANCHORS):
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'anchors'
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(x=x, y=y, z=z)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
            sphere.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0)
            markers.markers.append(sphere)
 
            text = Marker()
            text.header = sphere.header
            text.ns = 'anchors'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=x, y=y, z=z - 0.2)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f'A{i} ({x:.1f}, {y:.1f}, {z:.1f})'
            markers.markers.append(text)
 
        self.pub.publish(markers)
 
def main():
    rclpy.init()
    rclpy.spin(AnchorPublisher())
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 
