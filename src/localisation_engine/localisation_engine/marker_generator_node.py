import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from rclpy.executors import ExternalShutdownException

class MarkerGenerator(Node):

    def __init__(self):
        super().__init__("marker_generator_node")

        self.declare_parameter("tag_coord_topic", "/tag/coords")
        self.declare_parameter("tag_marker_topic", "/tag/marker")

        self.tag_marker = self.setup_marker()

        self.tag_marker_publisher = self.create_publisher(
            Marker,
            self.get_parameter("tag_marker_topic").get_parameter_value().string_value,
            10
        )

        self.tag_coords_subscriber = self.create_subscription(
            PointStamped,
            self.get_parameter("tag_coord_topic").get_parameter_value().string_value,
            self.tag_coords_subscribtion_cb,
            10
        )

    def tag_coords_subscribtion_cb(self, msg):
        self.tag_marker.header = msg.header
        self.tag_marker.pose.position = msg.point
        self.tag_marker_publisher.publish(self.tag_marker)

    def setup_marker(self) -> Marker:
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MarkerGenerator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()