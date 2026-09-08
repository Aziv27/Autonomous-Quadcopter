#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
import math

class PIDMixer(Node):
    def __init__(self):
        super().__init__('pid_mixer')
        
        self.create_subscription(Wrench, '/drone/thrust', self.thrust_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, qos_profile_sensor_data)
        
        # 1. NEW SUBSCRIPTION: Listen for live updates from our custom GUI
        self.create_subscription(Vector3, '/pid_gains', self.gains_callback, 10)
        
        self.pub_fr = self.create_publisher(Wrench, '/drone/motors/front_right', 10)
        self.pub_fl = self.create_publisher(Wrench, '/drone/motors/front_left', 10)
        self.pub_br = self.create_publisher(Wrench, '/drone/motors/back_right', 10)
        self.pub_bl = self.create_publisher(Wrench, '/drone/motors/back_left', 10)
        
        self.target_thrust = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        
        self.roll_integral = 0.0
        self.pitch_integral = 0.0
        
        # Starting Gains
        self.Kp = 2.5
        self.Ki = 0.1
        self.Kd = 1.5
        self.max_motor_thrust = 10.0
        
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("PID Mixer Online. Listening for thrust and live gains...")

    # 2. NEW CALLBACK: Update variables instantly when the slider moves
    def gains_callback(self, msg):
        self.Kp = msg.x
        self.Ki = msg.y
        self.Kd = msg.z
        # This logs to your terminal so you have absolute proof it updated!
        self.get_logger().info(f"Gains Updated: Kp={self.Kp:.2f} | Ki={self.Ki:.2f} | Kd={self.Kd:.2f}")

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        return roll, pitch

    def imu_callback(self, msg):
        q = msg.orientation
        self.current_roll, self.current_pitch = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

    def thrust_callback(self, msg):
        self.target_thrust = msg.force.z

    def send_motor_force(self, publisher, force):
        msg = Wrench()
        clamped_force = max(0.0, min(float(force), self.max_motor_thrust))
        msg.force.z = clamped_force
        publisher.publish(msg)

    def control_loop(self):
        if self.target_thrust <= 0.0:
            self.send_motor_force(self.pub_fr, 0.0)
            self.send_motor_force(self.pub_fl, 0.0)
            self.send_motor_force(self.pub_br, 0.0)
            self.send_motor_force(self.pub_bl, 0.0)
            self.roll_integral = 0.0
            self.pitch_integral = 0.0
            return

        roll_error = 0.0 - self.current_roll
        pitch_error = 0.0 - self.current_pitch
        
        self.roll_integral += roll_error * 0.01
        self.pitch_integral += pitch_error * 0.01

        # --- ANTI-WINDUP CLAMP ---
        max_integral = 5.0  # Limits the maximum accumulated memory
        self.roll_integral = max(-max_integral, min(max_integral, self.roll_integral))
        self.pitch_integral = max(-max_integral, min(max_integral, self.pitch_integral))
        # -------------------------
        
        # Calculate corrections
        roll_correction = (self.Kp * roll_error) + (self.Ki * self.roll_integral)
        pitch_correction = (self.Kp * pitch_error) + (self.Ki * self.pitch_integral)
        
        base = self.target_thrust / 4.0
        
        # CORRECTED MATH: Subtracting a negative correction increases the thrust.
        # If nose drops (pitch_correction is negative), front motors gain thrust.
        # If right side drops (roll_correction is negative), right motors gain thrust.
        fr_thrust = base - pitch_correction - roll_correction
        fl_thrust = base - pitch_correction + roll_correction
        br_thrust = base + pitch_correction - roll_correction
        bl_thrust = base + pitch_correction + roll_correction
        
        self.send_motor_force(self.pub_fr, fr_thrust)
        self.send_motor_force(self.pub_fl, fl_thrust)
        self.send_motor_force(self.pub_br, br_thrust)
        self.send_motor_force(self.pub_bl, bl_thrust)

def main(args=None):
    rclpy.init(args=args)
    node = PIDMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()