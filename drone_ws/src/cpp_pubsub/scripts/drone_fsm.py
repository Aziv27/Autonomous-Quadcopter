#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
from enum import Enum
import time
import math

class State(Enum):
    GROUND_IDLE = 1
    ASCENT = 2
    HOVER = 3
    FREEFALL = 4
    IMPACT = 5

class DroneFSM(Node):
    def __init__(self):
        super().__init__('drone_fsm')
        
        self.state = State.GROUND_IDLE
        self.current_altitude = 0.0
        self.filtered_z_accel = 9.81
        self.alpha = 0.1 
        self.hover_start_time = 0.0
        self.loop_count = 0 
        
        self.mass = 1.25666
        self.gravity = 9.81
        self.hover_thrust = self.mass * self.gravity
        self.ascent_thrust = 20.0
        
        self.thrust_pub = self.create_publisher(Wrench, '/drone/thrust', 10)
        
        self.create_subscription(LaserScan, '/sensor/sonar', self.sonar_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, '/drone/arm', self.arm_callback, 10)
        
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Drone FSM initialized. Waiting for ARM command on /drone/arm...")

        # Insert inside __init__(self):
        self.land_sub = self.create_subscription(Bool, '/drone/land', self.land_callback, 10)
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)
        self.last_heartbeat = self.get_clock().now()
        self.stm32_healthy = False  # Simulated hardware flag

    def land_callback(self, msg):
        if msg.data:
            current_state_str = str(self.state)
            # Reject if already on the ground
            if any(keyword in current_state_str for keyword in ["IDLE", "DISARMED", "INIT"]):
                self.get_logger().warn(f"ILLEGAL COMMAND REJECTED: Cannot land from state {current_state_str}")
            else:
                self.get_logger().info("Command accepted. Transitioning to LANDING.")
                self.state = "LANDING"

    def watchdog_callback(self):
        time_since = (self.get_clock().now() - self.last_heartbeat).nanoseconds / 1e9
        
        # Convert Enum state to a string to catch any naming variations
        current_state_str = str(self.state)
        
        # If the state string contains any safe keywords, ignore the heartbeat loss
        is_safe = any(keyword in current_state_str for keyword in ["IDLE", "DISARMED", "INIT", "FAILSAFE"])
        
        if time_since > 0.5 and not is_safe:
            self.get_logger().error(f"Heartbeat lost in state {current_state_str}! FORCING FAILSAFE LAND")
            
            # NOTE: If your script strictly requires setting an Enum (e.g., self.state = State.FAILSAFE_LAND), 
            # update the right side of the equation below to match your class structure.
            self.state = "FAILSAFE_LAND" 
            
            self.last_heartbeat = self.get_clock().now()

    def sonar_callback(self, msg):
        if len(msg.ranges) > 0:
            val = msg.ranges[0]
            if not math.isnan(val):
                if math.isinf(val) and val > 0:
                    self.current_altitude = 8.0  
                elif math.isinf(val) and val < 0:
                    self.current_altitude = 0.0  
                else:
                    self.current_altitude = val

    def imu_callback(self, msg):
        raw_z = msg.linear_acceleration.z
        self.filtered_z_accel = (self.alpha * raw_z) + ((1.0 - self.alpha) * self.filtered_z_accel)

    def arm_callback(self, msg):
        # The exact fix for your Kill Switch
        if msg.data and self.state == State.GROUND_IDLE:
            self.transition_to(State.ASCENT)
        elif not msg.data:
            self.get_logger().warn("DISARM COMMAND RECEIVED! EMERGENCY MOTOR KILL.")
            self.transition_to(State.GROUND_IDLE)

    def transition_to(self, new_state):
        self.get_logger().info(f"Transitioning from {self.state.name} to {new_state.name}")
        self.state = new_state

    def publish_thrust(self, z_force):
        msg = Wrench()
        msg.force.z = float(z_force)
        self.thrust_pub.publish(msg)

    def control_loop(self):
        self.loop_count += 1
        
        if self.state == State.GROUND_IDLE:
            self.publish_thrust(0.0)
            
        elif self.state == State.ASCENT:
            self.publish_thrust(self.ascent_thrust)
            
            if self.loop_count % 10 == 0:
                self.get_logger().info(f"Climbing... Alt: {self.current_altitude:.2f}m")
                
            if self.current_altitude >= 5.0:
                self.hover_start_time = time.time()
                self.transition_to(State.HOVER)
                
        elif self.state == State.HOVER:
            self.publish_thrust(self.hover_thrust)
            if time.time() - self.hover_start_time >= 3.0:
                self.transition_to(State.FREEFALL)
                
        elif self.state == State.FREEFALL:
            self.publish_thrust(0.0)  
            if self.filtered_z_accel > 25.0:
                self.transition_to(State.IMPACT)
                
        elif self.state == State.IMPACT:
            self.publish_thrust(0.0)
            self.get_logger().info("CRASH DETECTED VIA IMU SPIKE. Resetting to IDLE.")
            self.transition_to(State.GROUND_IDLE)

def main(args=None):
    rclpy.init(args=args)
    node = DroneFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()