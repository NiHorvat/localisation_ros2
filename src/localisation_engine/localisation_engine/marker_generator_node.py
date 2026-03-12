from custom_msgs.msg import PointStampedArray


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.executors import ExternalShutdownException

class MarkerGenerator(Node):

    def __init__(self):
        super().__init__("marker_generator_node")

        self.declare_parameter("tag_coord_topic", "/tag/coords")
        self.declare_parameter("tag_marker_topic", "/tag/marker")
        self.declare_parameter("anchor_coords_topic", "/anchors/coords")
        self.declare_parameter("anchor_marker_topic", "/anchors/marker")


        self.declare_parameter("marker_green", 0.0)
        self.declare_parameter("marker_red", 1.0)
        self.declare_parameter("marker_blue", 0.0)


        self.tag_marker_ = self.setup_tag_marker()
        

        self.tag_marker_publisher_ = self.create_publisher(
            Marker,
            self.get_parameter("tag_marker_topic").get_parameter_value().string_value,
            10
        )

        self.tag_coords_subscriber_ = self.create_subscription(
            PointStamped,
            self.get_parameter("tag_coord_topic").get_parameter_value().string_value,
            self.tag_coords_subscribtion_cb,
            10
        )
        
        self.anchor_coords_subscriber_ = self.create_subscription(
            PointStampedArray,
            self.get_parameter("anchor_coords_topic").get_parameter_value().string_value,
            self.anchor_coords_subscriontion_cb,
            10
        )
        
        self.anchor_marker_publisher_ = self.create_publisher(
            MarkerArray,
            self.get_parameter("anchor_marker_topic").get_parameter_value().string_value,
            10
        )
        



    def tag_coords_subscribtion_cb(self, msg):
        self.tag_marker_.header = msg.header
        self.tag_marker_.pose.position = msg.point
        self.tag_marker_publisher_.publish(self.tag_marker_)

    def anchor_coords_subscriontion_cb(self, msg: PointStampedArray):
        marker_array = MarkerArray()

        # Delete all previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, point_stamped in enumerate(msg.points):
            marker = Marker()
            marker.header = msg.header
            marker.ns = "anchors"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point_stamped.point.x
            marker.pose.position.y = point_stamped.point.y
            marker.pose.position.z = point_stamped.point.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.anchor_marker_publisher_.publish(marker_array)        


    def setup_tag_marker(self) -> Marker:
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = self.get_parameter("marker_red").get_parameter_value().double_value
        marker.color.g = self.get_parameter("marker_green").get_parameter_value().double_value
        marker.color.b = self.get_parameter("marker_blue").get_parameter_value().double_value
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