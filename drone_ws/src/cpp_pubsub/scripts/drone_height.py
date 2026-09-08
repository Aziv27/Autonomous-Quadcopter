#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
# THIS IS THE FIX: Import the Best Effort sensor profile
from rclpy.qos import qos_profile_sensor_data

class DroneDataBridge(Node):
    def __init__(self):
        super().__init__("drone_data_bridge")
        
        # Apply the sensor QoS profile so it matches Gazebo
        self.subscription = self.create_subscription(
            LaserScan,
            "/sensor/sonar",
            self.sonar_callback,
            qos_profile_sensor_data
        )
        self.alt_publisher = self.create_publisher(Float64, "/drone/altitude", 10)

    def sonar_callback(self, msg):
        raw_altitude = msg.ranges[0]
        
        clean_alt = Float64()

        # Hardware Filter
        if math.isinf(raw_altitude) or raw_altitude < msg.range_min or raw_altitude > msg.range_max:
            self.get_logger().warn(f"Out of bounds. Raw value: {raw_altitude}")
            
            # FIX: Publish a "Heartbeat" value (-1.0) so the network stays alive
            clean_alt.data = -1.0
            self.alt_publisher.publish(clean_alt)
            return
            
        clean_alt.data = float(raw_altitude)
        
        self.alt_publisher.publish(clean_alt)
        self.get_logger().info(f"Clean Altitude: {clean_alt.data:.3f} m")

def main():
    rclpy.init()
    node = DroneDataBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()