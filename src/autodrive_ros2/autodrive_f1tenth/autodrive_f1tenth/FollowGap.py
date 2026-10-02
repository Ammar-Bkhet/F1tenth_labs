#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from math import cos, sin
from numpy import arctan


class AEBNode(Node):

    def __init__(self):
        super().__init__("PID_Node")
        qos = QoSProfile(depth=1)
        self.subscriber = self.create_subscription(LaserScan,'/autodrive/f1tenth_1/lidar', self.laser_data_callback, 10)
        self.pub_steering = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.pub_throttle = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.get_logger().info("PID controller Node has started")
        self.prev_time = 0
        self.current_time= 0
        self.e_prev = 0
        self.integral = 0 
        

        
        

    def laser_data_callback(self, msg: LaserScan):
        msg.range_max = float('inf')
        # print(msg.angle_min)
        cmd1 = Float32()
        cmd2 = Float32()

        self.a = msg.ranges[359] # second ray -> angle to the right of car x_axis = -45 degree
        # zero is frontal
        self.b = msg.ranges[179] #first ray -> theta = -90 degree
        if self.a == float('inf'):
            self.a = 10.0
        if self.b == float('inf'):
            self.b = 10.0
        print(self.b)
        self.theta = 0.004351851996034384*180  # =45 degrees
    
        self.alpha = arctan((self.a*cos(self.theta)-self.b)/self.a * sin(self.theta))
        self.Dt = self.b * cos(self.alpha)
        self.L = 0.5
        self.Dt1 = self.Dt + self.L * sin(self.alpha)
        self.error = 0.6 - self.Dt1
        print(self.Dt1)
        # print(self.error)
        
    
        self.cmd1T = self.PID_controller(self.error,0.7,0.0,0.5) 
        if 1 < self.cmd1T :
            self.cmd1T = 1.0
        
        elif -1 > self.cmd1T:
            self.cmd1T = -1.0
            
     
        cmd1.data = self.cmd1T
        # print(cmd1.data)
        

        if 0<=abs(cmd1.data)<0.5 :
            cmd2.data= 0.05
           

        elif 0.5<=abs(cmd1.data)<=1 :
            cmd2.data = 0.05

        else :
            cmd2.data = 0.005
        
        self.pub_steering.publish(cmd1)
        self.pub_throttle.publish(cmd2)
        # print(cmd2.data)

        
    # def PID(self,Kp, Kd, Error):
        
        
        
    #     # PID calculations
        
    #     P = Kp*Error
        
    #     current_time = time.time()
    #     dt = current_time - self.prev_time
    #     print(dt)
    #     # integral = integral + Ki*e*(time - time_prev)
    #     D = Kd*((Error - self.e_prev)/(dt))# calculate manipulated variable - MV
    #     MV =  P + D
    #     # update stored data for next iteration
    #     self.e_prev = Error
        
    #     self.prev_time = current_time
    #     # self.current_time += 0.05
    #     return MV
    
    def PID_controller(self,error, Kp, Ki, Kd ):
        current_time = time.time()
        dt = current_time - self.prev_time
        print(dt)
        # Calculate the proportional term
        P_term = Kp * error

        # Calculate the integral term
        self.integral = self.integral + error * dt
        I_term = Ki * self.integral

        # Calculate the derivative term
        derivative = (error - self.e_prev) / dt
        D_term = Kd * derivative

        # Calculate the control output
        output = P_term + I_term + D_term

        # Update previous values
        
        self.e_prev = error
        
        self.prev_time = current_time

        return output
            
        
        


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
     main()