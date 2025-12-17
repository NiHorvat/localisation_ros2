
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class MarkerGenerator(Node):

    def __init__(self):
        super.__init__("marker_generator_node")



        self.declare_parameter("tag_coord_topic","/tag/coords")
        self.declare_parameter("tag_marker_topic","/tag/marker")

        self.tag_marker = self.setup_marker()

        self.tag_coords_subscriber = self.create_subscription(
            msg_type=Point,
            qos_profile=10,
            topic=self.get_parameter("tag_coord_topic").get_parameter_value().string_value,
            callback=self.tag_coords_subscriber
        )

        


        self.



    def tag_coords_subscribtion_cb(self, msg):
        ...
    

    def setup_marker() -> Marker:
        marker = Marker()

        marker.type = 




def main(args=None):

    rclpy.init(args=args)

    node = MarkerGenerator()
    try:
        rclpy.spin(node)

    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()