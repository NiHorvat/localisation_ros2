#! /bin/bash

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import numpy as np


## hardcoded anchor coordinates


class TrilaterationEngine(Node):
    def __init__(self):
        super().__init__("TrilaterationEngine")

        self.declare_parameter("distance_topic", "/distances")
        self.declare_parameter("tag_num", 3)



        self.distances_subscriber_ = self.create_subscription(
            msg_type=Float32MultiArray,
            topic=self.get_parameter("distance_topic").get_parameter_value().string_value,
            callback=self.distances_callback
        ) 



    def distances_callback(self, msg : Float32MultiArray):
        ...
        #calculate the position using trilateration
        
        


def main(args):
    rclpy.init(args=args)

    trilaterationEngine = TrilaterationEngine()

    rclpy.spin(trilaterationEngine)

    trilaterationEngine.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()