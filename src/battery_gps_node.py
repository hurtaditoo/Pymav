#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from pymavlink import mavutil
import time

class BatteryGPSNode(Node):
    def __init__(self):
        super().__init__('battery_gps_node')
        
        # Publisher for battery status
        self.battery_publisher = self.create_publisher(Int32, 'battery_status', 10)
        
        # Subscriber to target GPS position
        self.target_subscriber = self.create_subscription(
            Point, 'target_gps', self.target_callback, 10
        )
        
        # Connection to the drone simulator (SITL)
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to SITL")

        # Start a timer to periodically publish battery status
        self.timer = self.create_timer(1.0, self.publish_battery_status)

    def publish_battery_status(self):
        # Read battery status from the drone
        msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=False)
        if msg:
            battery_percentage = msg.battery_remaining
            battery_msg = Int32()
            battery_msg.data = battery_percentage
            self.battery_publisher.publish(battery_msg)
            self.get_logger().info(f"Battery status: {battery_percentage}%")
    
    def target_callback(self, msg):
        # Receive target GPS coordinates from the topic
        target_lat = msg.x
        target_lon = msg.y
        target_alt = msg.z
        
        # Send the target position to the drone
        self.go_to_position(target_lat, target_lon, target_alt)
    
    def go_to_position(self, lat, lon, alt):
        self.connection.mav.set_position_target_global_int_send(
            0,  # Timestamp
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b110111111000,   # Ignore velocity and acceleration
            int(lat * 1e7),   # Latitude
            int(lon * 1e7),   # Longitude
            alt,              # Altitude
            0, 0, 0,          # Velocity
            0, 0, 0,          # Acceleration
            0, 0              # Yaw and Yaw rate
        )
        self.get_logger().info(f"Moving drone to Lat: {lat}, Lon: {lon}, Alt: {alt} m")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryGPSNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()