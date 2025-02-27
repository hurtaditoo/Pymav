#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from pymavlink import mavutil
import time

# Function to read battery status message
def read_battery_status(connection):
    msg = connection.recv_match(type='BATTERY_STATUS', blocking=False, timeout=5)   

    if msg:                                         
        voltage = msg.voltages[0] / 1000.0          # Convert mv to V
        current = msg.current_battery / 100.0       # Convert cA to A
        remaining = msg.battery_remaining           # Remaining battery percentage
        print(f"Battery: {voltage:.2f} V, Current: {current:.2f} A, Remaining: {remaining} %")
        return remaining

    else:   # In case the message is not valid
        print("Failed to get battery status.")
        time.sleep(0.5)
        return None


# Class to create a node that reads battery status and moves the drone to a target GPS position
class BatteryGPSNode(Node):
    def __init__(self):
        super().__init__('battery_gps_node')        # Create a node with the name 'battery_gps_node'
        
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

    # Function to read battery status from the drone and publish it
    def publish_battery_status(self):
        battery_percentage = read_battery_status(self.connection)
        if battery_percentage is not None:

            battery_msg = Int32()
            battery_msg.data = battery_percentage           # Sending the battery percentage
            self.battery_publisher.publish(battery_msg)
                
    def target_callback(self, msg):
        # Receive target GPS coordinates from the topic
        target_lat = msg.x
        target_lon = msg.y
        target_alt = msg.z
        
        self.connection.set_mode('GUIDED')

        # Send the target position to the drone
        self.go_to_position(target_lat, target_lon, target_alt)
    
    # Function to move the drone to a specific position (latitude, longitude, altitude)
    def go_to_position(self, lat, lon, alt):
        self.connection.mav.set_position_target_global_int_send(
            0,  # Timestamp
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Relative altitude frame
            0b110111111000,                                 # Ignore velocity and acceleration
            int(lat * 1e7),                                 # Latitude as integer
            int(lon * 1e7),                                 # Longitude as integer
            alt,                                            # Desired altitude in meters
            0, 0, 0,                                        # Velocities
            0, 0, 0,                                        # Accelerations
            0, 0                                            # Yaw and Yaw rate
        )
        self.get_logger().info(f"Moving drone to Lat: {lat}, Lon: {lon}, Alt: {alt} m")


def main(args=None):    
    rclpy.init(args=args)
    node = BatteryGPSNode()
    rclpy.spin(node)    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
