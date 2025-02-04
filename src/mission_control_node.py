#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, String
from pymavlink import mavutil
import time
import sys

class MissionControlNode(Node):
    def __init__(self, target_lat, target_lon, target_alt):
        super().__init__('mission_control_node')

        # Publisher to the target_gps topic
        self.target_publisher = self.create_publisher(Point, 'target_gps', 10)

        # Subscriber for battery status
        self.battery_subscriber = self.create_subscription(
            Int32, 'battery_status', self.battery_callback, 10
        )
        
        # Connection to the drone simulator (SITL)
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to SITL")
        
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.target_alt = target_alt
        
        # Send the target GPS coordinates
        self.publish_target_gps()

    def publish_target_gps(self):
        target_msg = Point()
        target_msg.x = self.target_lat
        target_msg.y = self.target_lon
        target_msg.z = self.target_alt
        self.target_publisher.publish(target_msg)
        self.get_logger().info(f"Publishing target GPS: Lat: {self.target_lat}, Lon: {self.target_lon}, Alt: {self.target_alt} m")

    def battery_callback(self, msg):
        # Monitor battery status and initiate landing if below 20%
        battery_remaining = msg.data
        self.get_logger().info(f"Battery: {battery_remaining}%")
        if battery_remaining <= 20:
            self.initiate_landing()

    def initiate_landing(self):
        # Send the landing command to the drone
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0  # Additional parameters
        )
        self.get_logger().info("Battery low! Initiating emergency landing.")

def main(args=None):
    if len(sys.argv) < 4:
        print("Usage: mission_control_node.py <latitude> <longitude> <altitude>")
        sys.exit(1)

    target_lat = float(sys.argv[1])
    target_lon = float(sys.argv[2])
    target_alt = float(sys.argv[3])

    rclpy.init(args=args)
    node = MissionControlNode(target_lat, target_lon, target_alt)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()