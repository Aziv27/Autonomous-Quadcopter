#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUFilterNode(Node):
    def __init__(self):
        super().__init__('imu_filter')
        # Subscribe to the raw IMU data from Gazebo
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        
        self.filtered_z = 9.81
        # Alpha controls the smoothing. Lower = smoother but slightly more lag.
        self.alpha = 0.1 

    def imu_callback(self, msg):
        raw_z = msg.linear_acceleration.z
        
        # Apply the Low-Pass Filter
        self.filtered_z = (self.alpha * raw_z) + ((1.0 - self.alpha) * self.filtered_z)
        
        # Print the clean data to the terminal
        self.get_logger().info(f"Filtered Z-Accel: {self.filtered_z:.3f} m/s^2")

def main(args=None):
    rclpy.init(args=args)
    imu_filter = IMUFilterNode()
    try:
        rclpy.spin(imu_filter)
    except KeyboardInterrupt:
        pass
    finally:
        imu_filter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()