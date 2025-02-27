#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from pymavlink import mavutil
import time
import sys


# Class to create a node that moves the drone to a target GPS position
class MissionControlNode(Node):
    def __init__(self, target_lat, target_lon, target_alt):
        super().__init__('mission_control_node')

        # Publisher to the target_gps topic
        self.target_publisher = self.create_publisher(Point, 'target_gps', 10)

        # Subscriber to the target_gps topic
        self.target_subscriber = self.create_subscription(Int32, 'battery_status', self.battery_callback, 10)

        # Connection to the drone simulator (SITL)
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to SITL")

        self.target_lat = target_lat
        self.target_lon = target_lon
        self.target_alt = target_alt

        # Arm the drone and take off before sending the target
        self.arm_and_takeoff()


    def set_mode(self, mode):
        self.connection.set_mode(mode)                          # Change the flight mode of the drone
        self.get_logger().info(f"Mode changed to {mode}.")      # Print that the flight mode has changed

    def arm_and_takeoff(self):
        # Change the mode to GUIDED
        self.set_mode('GUIDED')

        # Arming the drone
        self.get_logger().info("Arming drone...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,                                                  # Confirmation (0 means no confirmation)
            1,                                                  # Arm the drone
            0, 0, 0, 0, 0, 0                                    # Other parameters are not needed for arming
        )

        # Wait for the drone to arm
        time.sleep(2)

        # Take off to the target altitude
        self.get_logger().info("Taking off...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,                                                  # Confirmation (0 means no confirmation)
            0, 0, 0, 0, 0, 0, 10                                # Target altitude
        )

        # Wait for takeoff to complete
        time.sleep(12)

    # Function called periodically to check battery status
    def battery_callback(self, msg):
        battery_remaining = msg.data

        if battery_remaining is not None:
            self.get_logger().info(f"Battery: {battery_remaining}%")
            if battery_remaining <= 20:
                self.initiate_landing()

        else:
            battery_remaining = None

    def initiate_landing(self):
        # Send the landing command to the drone
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,                                  # Confirmation
            0, 0, 0, 0, 0, 0, 0                 # Additional parameters
        )
        self.get_logger().info("Low battery! Initiating emergency landing.")
        time.sleep(30)

    def publish_target(self):
        # Publish the target GPS coordinates to the ROS2 topic
        target_point = Point()
        
        target_point.x = self.target_lat
        target_point.y = self.target_lon
        target_point.z = self.target_alt

        self.target_publisher.publish(target_point)
        self.get_logger().info(f"Published target GPS: Lat={self.target_lat}, Lon={self.target_lon}, Alt={self.target_alt} m")


def main(args=None):
    if len(sys.argv) < 4:
        print("Usage: mission_control_node.py <latitude> <longitude> <altitude>")
        sys.exit(1)

    target_lat = float(sys.argv[1])
    target_lon = float(sys.argv[2])
    target_alt = float(sys.argv[3])     

    rclpy.init(args=args)
    node = MissionControlNode(target_lat, target_lon, target_alt)
    node.publish_target()  
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
