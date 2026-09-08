#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class IMUChecker(Node):
    def __init__(self):
        super().__init__('imu_checker')
        self.sub = self.create_subscription(Imu, '/imu/data', self.callback, 10)
        self.count = 0

    def callback(self, msg):
        self.count += 1
        # If IMU publishes at 250Hz, skipping 50 prints exactly 5 times a second
        if self.count % 50 != 0:
            return
        
        q = msg.orientation
        
        # Convert Quaternion to Roll
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.degrees(math.atan2(t0, t1))
        
        # Convert Quaternion to Pitch
        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.degrees(math.asin(t2))
        
        print(f"Nose Pitch: {pitch:6.1f}°  |  Right Roll: {roll:6.1f}°")

def main():
    rclpy.init()
    rclpy.spin(IMUChecker())
    rclpy.shutdown()

if __name__ == '__main__':
    main()