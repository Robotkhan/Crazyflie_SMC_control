from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from builtin_interfaces.msg import Time
from std_msgs.msg import Float64

import rclpy
from rclpy.node import Node
import datetime as dt


class PIDcontrol(Node):
    def __init__(self):
        super().__init__("PID_control_node")
        
        self.prev_time = self.get_clock().now()

        self.cmd_vel_pub = self.create_publisher(Twist, "/crazyflie/cmd_vel", 10)
        self.cmd_vel_sub = self.create_subscription(Odometry, "/crazyflie/odom", self.cmd_vel_callback, 10)
        self.altitude_pub = self.create_publisher(Float64, '/altitude', 10)

        self.desired_altitude = 1.5
        self.Kp = 1.0
        self.Ki = 0.005
        self.Kd = 0.05

        # smc parameters:
        self.Ksmc = 0.5
        self.lambda_smc = 0.01
    
        self.altitude_record_file = open('altitude_data.txt', 'w')
    
    
    def pid_control(self, desired_signal, actual_signal, timestep=0.02, Kp=1.0, Ki=0.005, Kd=0.05):
        error = desired_signal - actual_signal
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral += error
        control_signal = Kp * error + Ki * self.integral + Kd * (error - self.prev_error) / timestep
        self.prev_error = error
        
        return control_signal
    
    def sliding_mode_control(self, desired_signal, actual_signal, Kgain, lambda_, timestep=0.02):
        error = desired_signal - actual_signal
        error_dot = error / timestep
        sliding_surface = error_dot + lambda_ * error
        control_signal = Kgain * np.sign(sliding_surface)
        return control_signal   

    def cmd_vel_callback(self, msg):
        vel = Twist()
        alt = Float64()
        
        alt.data = msg.pose.pose.position.z
        self.altitude_record_file.write(f'{alt}\n')
        
        error = self.desired_altitude - msg.pose.pose.position.z 
        
        current_time = self.get_clock().now()
        timestep = (current_time - self.prev_time).nanoseconds / 1e9
        # pid_control = self.pid_control(self.desired_altitude, alt.data, timestep, self.Kp, self.Ki, self.Kd)
        sliding_mode_control = self.sliding_mode_control(self.desired_altitude, alt.data, self.Ksmc, self.lambda_smc, timestep)
        self.prev_time = current_time
    
        # vel.linear.z = pid_control
        vel.linear.z = sliding_mode_control
        vel.linear.x = 0.0
        vel.linear.y = 0.0

        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0

        self.get_logger().info(f"Current altitude error: {error}/n Current altitude: {msg.pose.pose.position.z}")
        self.get_logger().info(f'Timestep: {timestep}')

        self.cmd_vel_pub.publish(vel)
        self.altitude_pub.publish(alt)
    
    def destroy_node(self):
        self.altitude_record_file.close()
        super().destroy_node()
        
    
def main(args=None):
    rclpy.init(args=args)
    controller_node = PIDcontrol()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()