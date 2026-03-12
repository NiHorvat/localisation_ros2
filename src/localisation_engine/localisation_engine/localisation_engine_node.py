#! /bin/bash

from custom_msgs.msg import Distances, PointStampedArray
import localisation_engine.trilaterator as tril
import localisation_engine.EKF_raw as EKF_raw
import localisation_engine.EKF_w_calibration as EKF_w_calibration



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

        # functionc pointer - or what the python version of that name is called
        # depending on the implementation - Calibration/No calibration the impllementation of this function will be different
        self.get_anchor_coords_ : function = None


        # topic parameters    
        self.declare_parameter("distance_topic", "/distances")
        self.declare_parameter("tag_coord_topic","/tag/coords")
        self.declare_parameter("anchors_coords_topic", "/anchors/coords")
        
        
        # EKF noise parameters
        self.declare_parameter("robot_process_noise_stdev",0.1)
        self.declare_parameter("measurement_noise_stdev",0.1)
        self.declare_parameter("anchor_process_noise_stdev",0.0)

        # selectors for EKF type #TODO fix magic strings
        self.declare_parameter("EKF_TYPE", "EKF_raw_no_vel")
        self.declare_parameter("EKF_RANGING_MODE","ASYNC")

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
        
        self.anchor_coord_publisher_ = self.create_publisher(
            msg_type=PointStampedArray,
            qos_profile=10,
            topic=self.get_parameter("anchors_coords_topic").get_parameter_value().string_value
        )
        


        anchor_coords_flat = self.get_parameter("anchor_positions_param").get_parameter_value().double_array_value
        if(len(anchor_coords_flat) < 9):
            self.get_logger().error(f"Anchor count less than 3, unable to localise anchors{self.get_parameter("anchor_positions_param").get_parameter_value().double_array_value}")
            exit(0)


        self.anchor_coords_ = np.reshape(a=anchor_coords_flat, newshape=(len(anchor_coords_flat) // 3, 3))
        self.n_anchors_ = len(self.anchor_coords_)
        self.get_logger().info(
            f"Anchor Coordinates:\n {self.anchor_coords_}"
        )
        

        
        





        # Temp variables used as abreviations
        TEMP_ekf_type = self.get_parameter("EKF_TYPE").get_parameter_value().string_value
        TEMP_robot_q_stdev = self.get_parameter("robot_process_noise_stdev").get_parameter_value().double_value
        TEMP_anchor_q_stdev = self.get_parameter("anchor_process_noise_stdev").get_parameter_value().double_value

        TEMP_r_stdev = self.get_parameter("measurement_noise_stdev").get_parameter_value().double_value
        
        
        if(TEMP_ekf_type == "EKF_raw_no_vel"):
            self.ekf_c_ = EKF_raw.EKF_raw_no_vel(mode = self.get_parameter("EKF_RANGING_MODE").get_parameter_value().string_value, 
                        anchors=self.anchor_coords_, 
                        q_std=TEMP_robot_q_stdev, r_std=TEMP_r_stdev)
            self.get_anchor_coords_ = self.get_anchor_coords_EKF_no_callibration

        elif(TEMP_ekf_type == "EKF_raw_w_vel"):
            self.ekf_c_ = EKF_raw.EKF_raw_w_vel(mode = self.get_parameter("EKF_RANGING_MODE").get_parameter_value().string_value, 
                        anchors=self.anchor_coords_, 
                        q_std=TEMP_robot_q_stdev, r_std=TEMP_r_stdev)
            self.get_anchor_coords_ = self.get_anchor_coords_EKF_no_callibration
        elif(TEMP_ekf_type == "EKF_w_callibration"):
            self.ekf_c_ = EKF_w_calibration.EKF_w_callibration(mode = self.get_parameter("EKF_RANGING_MODE").get_parameter_value().string_value, 
                        anchors=self.anchor_coords_, q_robot_stdev=TEMP_robot_q_stdev,
                        q_anchor_stdev=TEMP_anchor_q_stdev, r_std=TEMP_r_stdev)
            
            self.get_anchor_coords_ = self.get_anchor_coords_EKF_w_callibration
        else: 
            self.get_logger().error(f"EKF_TYPE not selected properly EKF_TYPE={TEMP_ekf_type}")
            exit(1)


    def get_anchor_coords_EKF_no_callibration(self, x : np.ndarray):
        return self.anchor_coords_


    def get_anchor_coords_EKF_w_callibration(self,x : np.ndarray):
        # x is the EKF state vector and it includes the
        self.anchor_coords_ = x[6:]     # in state vector following the x,y,z and vx, vy, vz are the x,y,z for all the anchors
        return self.anchor_coords_


    def distances_callback(self, msg : Distances):
        
        if(msg is None or msg.distances is None):
            self.get_logger().error("msg or msg.distances is None")
            return
        
        new_state = self.ekf_c_.get_new_state(distances=msg)

        #self.get_logger().info(f"received distances{msg.distances}")
        self.get_logger().info(f"new state : {new_state}")     


        self.publish_tag_point(new_state[0:3])

        # depending on the mode of operation new_state has different shape/size
        new_anchor_coords  = self.get_anchor_coords_(new_state)
        self.publish_anchor_points(new_anchor_coords)


    def publish_tag_point(self, new_point_NP):
        
        if(new_point_NP is None):
            self.get_logger().error("new_point_NP is None")
            return
        
        new_point = PointStamped()
        new_point.header.stamp = self.get_clock().now().to_msg()
        new_point.header.frame_id = "map"

        new_point.point.x = float(new_point_NP[0])
        new_point.point.y = float(new_point_NP[1])
        new_point.point.z = float(new_point_NP[2])



        self.tag_coord_publisher_.publish(new_point)

        self.get_logger().info(f"Published new point {new_point}")

    def publish_anchor_points(self, anchor_coords_NP: np.ndarray):
        if anchor_coords_NP is None:
            self.get_logger().error("anchor_coords_NP is None")
            return

        coords = anchor_coords_NP.reshape(-1, 3)  # (n, 3) regardless of input shape

        msg = PointStampedArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for coord in coords:
            ps = PointStamped()
            ps.header = msg.header
            ps.point.x = float(coord[0])
            ps.point.y = float(coord[1])
            ps.point.z = float(coord[2])
            msg.points.append(ps)

        self.anchor_coord_publisher_.publish(msg)


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