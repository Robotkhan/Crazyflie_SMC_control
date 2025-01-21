from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from builtin_interfaces.msg import Time
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray

import rclpy
from rclpy.node import Node
import datetime as dt


class PIDcontrol(Node):
    def __init__(self):
        super().__init__("PID_control_node")
        
        self.prev_time = self.get_clock().now()

        self.cmd_vel_pub = self.create_publisher(Twist, "/crazyflie/cmd_vel", 10)
        self.cmd_vel_sub = self.create_subscription(Odometry, "/crazyflie/odom", self.cmd_vel_callback, 10)

        self.desired_altitude = 1.5
        self.Kp = 1.0
        self.Ki = 0.005
        self.Kd = 0.05
        self.mass = 27e-3
        self.g = 9.81
        self.Ixx=16.571710E-06
        self.Iyy=16.655602E-06 
        self.Izz=29.261652E-06
        self.Ixy=0.830806E-06 
        self.Ixz=0.718277E-06
        self.Iyz=1.800197E-06
        self.l_ = np.sqrt(0.031*0.031 + 0.031*0.031)
        self.Jr = 3.357e-9
        self.drag_constant = 8.06428e-05 # drag constant of the rotors
        self.thrust_constant = 2.3e-8 # thrust constant

        # smc parameters:
        
        # position gains
        self.Ksmc_x = 0.5
        self.Ksmc_y = 0.8
        self.Ksmc_z = 0.9

        # orientation gains
        self.Ksmc_roll = 1.5
        self.Ksmc_pitch = 1.5
        self.Ksmc_yaw = 1.5

        
        self.lambda_x = 20.0
        self.lambda_y = 1.0
        self.lambda_z = 5.0

        self.bx = 1/self.mass
        self.by = 1/self.mass
        self.bz = 1/self.mass

        self.b_roll = 1/self.Ixx
        self.b_pitch = 1/self.Iyy
        self.b_yaw = 1/self.Izz


        self.lambda_roll = 0.5
        self.lambda_pitch = 0.5
        self.lambda_yaw = 0.5
        
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0

        self.prev_error_roll = 0.0
        self.prev_error_pitch = 0.0
        self.prev_error_yaw = 0.0

        self.total_rotor_speed = 0.0

        self.AllocationMatrix = np.array([[self.thrust_constant, self.thrust_constant, self.thrust_constant, self.thrust_constant],
                                         [self.thrust_constant, 0, -self.thrust_constant, 0],
                                         [0, -self.thrust_constant, 0, self.thrust_constant],
                                         [-self.drag_constant, self.drag_constant, -self.drag_constant, self.drag_constant]])

        self.data_record_file = open('flight_data.txt', 'w')

    def pid_control(self, desired_signal, actual_signal, timestep=0.02, Kp=1.0, Ki=0.005, Kd=0.05):
        error = desired_signal - actual_signal
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral += error
        control_signal = Kp * error + Ki * self.integral + Kd * (error - self.prev_error) / timestep
        self.prev_error = error
        
        return control_signal

    def smc_altitude(self, desired_signal, actual_signal, Kgain, lambda_, timestep = 0.02):
        error = desired_signal - actual_signal
        error_z_dot = (error - self.prev_error_z) / timestep
        self.prev_error_z = error
        sliding_surface_z = error_z_dot + lambda_ * error
        u_eq = self.mass * (-self.g + error_z_dot * lambda_)
        control_signal = Kgain * np.sign(sliding_surface_z) + u_eq
        return control_signal 
    
    def sliding_mode_controller(self, desired_x, desired_y, desired_z, actual_x, actual_y, actual_z, desired_roll, desired_pitch, 
                                desired_yaw, actual_roll, actual_pitch, actual_yaw, roll_rate, pitch_rate, yaw_rate, timestep=0.02):
        error_x = desired_x - actual_x
        error_y = desired_y - actual_y
        error_z = desired_z - actual_z
        
        error_x_dot = (error_x - self.prev_error_x) / timestep
        error_y_dot = (error_y - self.prev_error_y) / timestep
        error_z_dot = (error_z - self.prev_error_z) / timestep

        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z

        s_x = self.lambda_x * error_x + error_x_dot
        s_y = self.lambda_y * error_y + error_y_dot
        s_z = self.lambda_z * error_z + error_z_dot

        u_eqx = self.mass * self.lambda_x * error_x_dot
        u_eqy = self.mass * self.lambda_y * error_y_dot
        u_eqz = self.mass * (self.g + error_z_dot * self.lambda_z)

        u_smc_x = self.Ksmc_x * np.sign(s_x) + u_eqx
        u_smc_y = self.Ksmc_y * np.sign(s_y) + u_eqy
        u_smc_z = self.Ksmc_z * np.sign(s_z) + u_eqz

        error_roll = desired_roll - actual_roll
        error_pitch = desired_pitch - actual_pitch
        error_yaw = desired_yaw - actual_yaw
        
        error_roll_dot = (error_roll - self.prev_error_roll) / timestep
        error_pitch_dot = (error_pitch - self.prev_error_pitch) / timestep
        error_yaw_dot = (error_yaw - self.prev_error_yaw) / timestep

        self.prev_error_roll = error_roll
        self.prev_error_pitch = error_pitch
        self.prev_error_yaw = error_yaw

        s_roll = self.lambda_roll * error_roll + error_roll_dot
        s_pitch = self.lambda_pitch * error_pitch + error_pitch_dot
        s_yaw = self.lambda_yaw * error_yaw + error_yaw_dot
 
        u_eqroll = (self.Ixx / self.l_) * (0 - pitch_rate * yaw_rate * ((self.Iyy - self.Ixx) / self.Ixx) + (self.Jr / self.Ixx) * pitch_rate * self.total_rotor_speed + self.lambda_roll * error_roll_dot)
        u_eqpitch = (self.Iyy / self.l_) * (0 - roll_rate * yaw_rate * ((self.Izz - self.Ixx) / self.Iyy) - (self.Jr * roll_rate * self.total_rotor_speed) / self.Iyy + self.lambda_pitch * error_pitch_dot)
        u_eqyaw = self.Izz * (0 - pitch_rate * roll_rate * ((self.Ixx - self.Iyy) / self.Izz) + self.lambda_yaw * error_yaw_dot)

        u_smc_roll = self.Ksmc_roll * np.sign(s_roll) + u_eqroll
        u_smc_pitch = self.Ksmc_pitch * np.sign(s_pitch) + u_eqpitch
        u_smc_yaw = self.Ksmc_yaw * np.sign(s_yaw) + u_eqyaw

        #calculation of u1, ux, and uy
        u1 = u_smc_z / (np.cos(actual_pitch) * np.cos(actual_roll))
        ux = u_smc_x / self.bx
        uy = u_smc_y / self.by

        #calculation of desired roll and pitch
        pitch_desired = np.arctan((ux * np.cos(desired_yaw) + uy * np.sin(desired_yaw))/(u_smc_z))
        roll_desired = np.arctan((np.cos(pitch_desired) * (ux * np.sin(desired_yaw) - uy * np.cos(desired_yaw)))/(u_smc_z))
        
        # calculation of u2, u3, and u4
        u2 = u_smc_roll / self.b_roll
        u3 = u_smc_pitch / self.b_pitch
        u4 = u_smc_yaw / self.b_yaw

        desired_inputs = np.array([u1, u2, u3, u4])

        omega_squared = np.linalg.solve(self.AllocationMatrix, desired_inputs)

        desired_rotor_speeds = np.sqrt(omega_squared)

        self.total_rotor_speed = sum(desired_rotor_speeds)

        return [u1, ux, uy, u2, u3, u4, pitch_desired, roll_desired] 
    

    def cmd_vel_callback(self, msg):
        vel = Twist()
        altitude = Float64()
        x = Float64()
        y = Float64()
        
        altitude.data = msg.pose.pose.position.z
        x.data = msg.pose.pose.position.x
        y.data = msg.pose.pose.position.y
        self.data_record_file.write(f'{x, y, altitude}\n')
        alt_error = self.desired_altitude - msg.pose.pose.position.z
        
        positions = msg.pose.pose.position
        actual_x_pos = positions.x
        actual_y_pos = positions.y
        actual_z_pos = positions.z


        orientation_q = msg.pose.pose.orientation
        actual_roll = orientation_q.x
        actual_pitch = orientation_q.y
        actual_yaw = orientation_q.z

        orientation_rates = msg.twist.twist.angular
        roll_rate = orientation_rates.x
        pitch_rate = orientation_rates.y
        yaw_rate = orientation_rates.z

        current_time = self.get_clock().now()
        timestep = (current_time - self.prev_time).nanoseconds / 1e9
        desired_outputs = self.sliding_mode_controller(0.0, 0.0, 1.5, actual_x_pos, actual_y_pos, actual_z_pos, 0.0, 0.0, 0.0, actual_roll, actual_pitch, actual_yaw, roll_rate, pitch_rate, yaw_rate, timestep)
        self.prev_time = current_time
    
        # altitude smc control
        

        vel.linear.z = desired_outputs[0] # u1
        vel.linear.x = desired_outputs[1] # ux
        vel.linear.y = desired_outputs[2] # uy

        vel.angular.x = desired_outputs[3] # u2
        vel.angular.y = desired_outputs[4] # u3
        vel.angular.z = desired_outputs[5] # u4

        self.get_logger().info(f"Current altitude error: {alt_error}/n Current altitude: {msg.pose.pose.position.z}")
        self.get_logger().info(f'Timestep: {timestep}')

        self.cmd_vel_pub.publish(vel)
    
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