#! /bin/bash

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import paho.mqtt.client as mqtt
from geometry_msgs.msg import PointStamped

import threading


class MQTT_client(Node):
    def __init__(self):
        super().__init__("MQTT_client_publisher")

        """
            Parameters concerning MQTT client
        """
        self.declare_parameter("mqtt_host","10.98.186.228")
        self.declare_parameter("mqtt_host_port",1883)
        self.declare_parameter("mqtt_clean_start",True)
        self.declare_parameter("mqtt_keep_alive",60)
        self.declare_parameter("mqtt_publish_topic","/tag/coords")
        self.declare_parameter("mqtt_qos",0)


        self.declare_parameter("tag_coord_topic","/tag/coords")


        self.mqttc = mqtt.Client(
            protocol=mqtt.MQTTv5 
        )
        self.mqttc.connect(
            host=self.get_parameter("mqtt_host").get_parameter_value().string_value,
            port=self.get_parameter("mqtt_host_port").get_parameter_value().integer_value,
            clean_start=self.get_parameter("mqtt_clean_start").get_parameter_value().bool_value,
            keepalive=self.get_parameter("mqtt_keep_alive").get_parameter_value().integer_value,
        )


        self.create_subscription(
            topic=self.get_parameter("tag_coord_topic").get_parameter_value().string_value,
            callback=self._on_tag_pos_cb,
            msg_type=PointStamped,
            qos_profile=10
        )        





    def _on_tag_pos_cb(self, msg : PointStamped):
        
        self.mqttc.publish(
            topic=self.get_parameter("mqtt_publish_topic").get_parameter_value().string_value,
            qos=self.get_parameter("mqtt_qos").get_parameter_value().integer_value,
            payload=str(msg)
        )

        self.get_logger().info(f"Published msg : {msg}")



        self.mqtt_thread = threading.Thread(target=self.mqttc.loop_start)
        self.mqtt_thread.daemon = True  
        self.mqtt_thread.start()

        


def main(args=None):

    rclpy.init(args=args)
    node = MQTT_client()
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