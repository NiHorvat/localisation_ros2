#! /bin/bash

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import paho.mqtt.client as mqtt
from geometry_msgs.msg import Point

import threading
import json
import ast



class MQTT_client(Node):
    def __init__(self):
        super().__init__("MQTT_client")

        """
            Parameters concerning MQTT client
        """
        self.declare_parameter("mqtt_host","10.98.186.228")
        self.declare_parameter("mqtt_host_port",1883)
        self.declare_parameter("mqtt_clean_start",True)
        self.declare_parameter("mqtt_keep_alive",60)
        self.declare_parameter("mqtt_subscriber_node","/tag/coords")
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




        self.tag_coord_publisher = self.create_publisher(
            topic=self.get_parameter("tag_coord_topic").get_parameter_value().string_value,
            msg_type=Point,
            qos_profile=10
        )        

        self.mqttc.subscribe(
            topic=self.get_parameter("mqtt_subscriber_node").get_parameter_value().string_value,
            qos=self.get_parameter("mqtt_qos").get_parameter_value().integer_value
        )

        def _on_message(client, userdata, msg : mqtt.MQTTMessage):
            try:
                clean_str = msg.payload.decode('utf-8')

                self.get_logger().info(str(clean_str))

                data_str = clean_str.split('Point(')[1].rstrip(')')

                kv_pairs = [pair.split('=') for pair in data_str.split(', ')]
                coords = {k: float(v) for k, v in kv_pairs}

                new_point = Point()
                # -10000 is for development purposes
                new_point.x = coords.get('x', -10000.0)
                new_point.y = coords.get('y', -10000.0)
                new_point.z = coords.get('z', -10000.0)


                self.tag_coord_publisher.publish(new_point)

            except Exception as e:
                self.get_logger().error(f"Error in on_message: {e}")


        self.mqttc.on_message = _on_message


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