#! /bin/bash

from custom_msgs.msg import Distances
import localisation_engine.trilaterator as tril

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped



import numpy as np


## hardcoded anchor coordinates


class LocalisationEngines(Node):
    def __init__(self):
        super().__init__("localisation_engine_node")

        self.declare_parameter("distance_topic", "/distances")
        self.declare_parameter("tag_coord_topic","/tag/coords")


        anch_pos_param_desc = ParameterDescriptor(
            description="Array of 3D positions of the anchor, x1,y1,z1, x2,y2,z2 ...",
            type=Parameter.Type.DOUBLE_ARRAY
        )
        self.declare_parameter(name="anchor_positions_param",
                               descriptor=anch_pos_param_desc,
                               value=[0.0])


        self.distances_subscriber_ = self.create_subscription(
            msg_type=Distances,
            topic=self.get_parameter("distance_topic").get_parameter_value().string_value,
            callback=self.distances_callback,
            qos_profile=10
        ) 
        
        self.tag_coord_publisher = self.create_publisher(
            msg_type=PointStamped,
            qos_profile=10,
            topic=self.get_parameter("tag_coord_topic").get_parameter_value().string_value
        )


        if(len(self.get_parameter("anchor_positions_param").get_parameter_value().double_array_value) < 9):
            self.get_logger().error(f"Anchor count less than 3, unable to localise anchors{self.get_parameter("anchor_positions_param").get_parameter_value().double_array_value}")
            exit(0)

        self.achor_coords = np.reshape(
            a=self.get_parameter("anchor_positions_param").get_parameter_value().double_array_value,
            newshape=(3,-1)
        )
        self.get_logger().info(
            f"Anchor Coordinates:\n {self.achor_coords}"
        )

        
        # trilateratod instance
        try:
            self.trilaterator = tril.Trilaterator(anchor_coords=self.achor_coords)
        except(Exception):
            self.get_logger().error("Trilaterator failed to load")
            exit(0)



    def distances_callback(self, msg : Distances):
        
        #TODO fix this with real trilateration calculation that takes in the account the errors that might occur, inaccuracy during the meassurements
        points = self.trilaterator.a(distances=msg.distances)

        self.get_logger().info(f"received distances{msg.distances}")
        self.get_logger().info(f"trilaterated : {points}")
        
        new_point = PointStamped()
        new_point.header.stamp = self.get_clock().now().to_msg()
        new_point.header.frame_id = "map"
        #TODO Placeholder if the point is null, REMOVE BEFORE FLIGHT
        if(points is not None):
            new_point.point.x = points[0][0]
            new_point.point.y = points[0][1]
            new_point.point.z = points[0][2]
        else:
            new_point.point.x = 1.0
            new_point.point.y = 1.0
            new_point.point.z = 0.0


        self.tag_coord_publisher.publish(new_point)



def main(args=None):

    rclpy.init(args=args)

    node = LocalisationEngines()
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