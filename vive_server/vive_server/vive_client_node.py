#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

import sys

from vive_server.vive_client import ViveClient, ViveConfig
from vive_server.models import TrackerState

class ViveNode(Node):
    def __init__(self):
    
        super().__init__('vive_tracker_node')
        self.declare_parameter('host_ip', '239.0.0.0')
        self.declare_parameter('host_port', 5007)
        self.declare_parameter('tracker_name', 'T_1')
        self.declare_parameter('topic', '')
        self.declare_parameter('link_name', 'odom')
        self.declare_parameter('child_link_name', 'tracker_link')

        (self.host_ip, self.host_port, self.tracker_name, self.link_name, self.child_link_name, self.topic) = self.get_parameters(
            ['host_ip', 'host_port', 'tracker_name', 'link_name', 'child_link_name', 'topic'])
        
        topic = self.topic.get_parameter_value().string_value
        topic_name = self.tracker_name.get_parameter_value().string_value + '/odom' if topic == "" else topic
        self.odom_pub = self.create_publisher(Odometry, topic_name,
            qos_profile=qos_profile_sensor_data)
        
        
        self.client_config = ViveConfig()
        self.client_config.address = self.host_ip.get_parameter_value().string_value
        self.client_config.port    = self.host_port.get_parameter_value().integer_value
        self.client_config.label   = self.tracker_name.get_parameter_value().string_value
       
        self.get_logger().info(str(self.client_config))

        self.client  = ViveClient(self.client_config)
        self.state = TrackerState()
        self.clock = self.get_clock()
        
        self.update_timer = self.create_timer(0.01, self.step)

    
    def step(self):
        if not rclpy.ok():
            sys.exit(0)
    
        msg = self.client.step()
        if msg:
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.link_name.get_parameter_value().string_value

            odom_msg.child_frame_id = self.child_link_name.get_parameter_value().string_value

            odom_msg.pose.pose.position.x = msg.xi
            odom_msg.pose.pose.position.y = msg.xj
            odom_msg.pose.pose.position.z = msg.xk

            odom_msg.pose.pose.orientation.x = msg.qi
            odom_msg.pose.pose.orientation.y = msg.qj
            odom_msg.pose.pose.orientation.z = msg.qk
            odom_msg.pose.pose.orientation.w = msg.qr

            odom_msg.twist.twist.linear.x = msg.v1
            odom_msg.twist.twist.linear.y = msg.v2
            odom_msg.twist.twist.linear.z = msg.v3

            odom_msg.twist.twist.angular.x = msg.w1
            odom_msg.twist.twist.angular.y = msg.w2
            odom_msg.twist.twist.angular.z = msg.w3

            self.odom_pub.publish(odom_msg)
            

def main(args=None):
    rclpy.init(args=args)
    node = ViveNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
