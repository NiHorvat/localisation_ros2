#! /bin/bash

from custom_msgs.msg import Distances
import localisation_engine.trilaterator as tril
import localisation_engine.EKF_raw as EKF_raw




import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped



import numpy as np




class LocalisationEngines(Node):
    def __init__(self):
        super().__init__("localisation_engine_node")

        self.declare_parameter("distance_topic", "/distances")
        self.declare_parameter("tag_coord_topic","/tag/coords")

        self.declare_parameter("robot_process_noise_stdev",0.1)
        self.declare_parameter("measurement_noise_stdev",0.1)



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
        
        self.tag_coord_publisher_ = self.create_publisher(
            msg_type=PointStamped,
            qos_profile=10,
            topic=self.get_parameter("tag_coord_topic").get_parameter_value().string_value
        )


        if(len(self.get_parameter("anchor_positions_param").get_parameter_value().double_array_value) < 9):
            self.get_logger().error(f"Anchor count less than 3, unable to localise anchors{self.get_parameter("anchor_positions_param").get_parameter_value().double_array_value}")
            exit(0)

        self.achor_coords_ = np.reshape(
            a=self.get_parameter("anchor_positions_param").get_parameter_value().double_array_value,
            newshape=(4,-1) #FIXME PRIORITY FR FR
        )
        self.get_logger().info(
            f"Anchor Coordinates:\n {self.achor_coords_}"
        )

        self.declare_parameter("EKF_TYPE", "EKF_raw_no_vel")
        
        self.declare_parameter("EKF_RANGING_MODE","ASYNC")


        TEMP_ekf_type = self.get_parameter("EKF_TYPE").get_parameter_value().string_value
        TEMP_robot_q_stdev = self.get_parameter("robot_process_noise_stdev").get_parameter_value().double_value
        TEMP_r_stdev = self.get_parameter("measurement_noise_stdev").get_parameter_value().double_value
        if(TEMP_ekf_type == "EKF_raw_no_vel"):
            self.ekf_c_ = EKF_raw.EKF_raw_no_vel(mode = self.get_parameter("EKF_RANGING_MODE").get_parameter_value().string_value, 
                                    anchors=self.achor_coords_, 
                                    q_std=TEMP_robot_q_stdev, r_std=TEMP_r_stdev)
        elif(TEMP_ekf_type == "EKF_raw_w_vel"):
            self.ekf_c_ = EKF_raw.EKF_raw_w_vel(mode = self.get_parameter("EKF_RANGING_MODE").get_parameter_value().string_value, 
                        anchors=self.achor_coords_, 
                        q_std=TEMP_ekf_type, r_std=TEMP_r_stdev)
        else: 
            self.get_logger().error(f"EKF_TYPE not selected properly EKF_TYPE={TEMP_ekf_type}")
            exit(1)


    def distances_callback(self, msg : Distances):
        
        point = self.ekf_c_.get_new_state(distances=msg)[0:3]

        #self.get_logger().info(f"received distances{msg.distances}")
        self.get_logger().info(f"new position : {point}")
        
        new_point = PointStamped()
        new_point.header.stamp = self.get_clock().now().to_msg()
        new_point.header.frame_id = "map"

        if(point is None):
            self.get_logger().error("Failed to calculate the new point")
            return
        
        new_point.point.x = float(point[0])
        new_point.point.y = float(point[1])
        new_point.point.z = float(point[2])

        self.tag_coord_publisher_.publish(new_point)



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