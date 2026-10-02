#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node 
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from math import cos, sin
from numpy import arctan
import time




class AEBNode(Node):

    def __init__(self):
        super().__init__("PID_Node")
        self.subscriber = self.create_subscription(LaserScan,"/scan", self.laser_data_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, "/drive", 5)
        self.get_logger().info("PID controller Node has started")
        self.prev_time = 0
        self.current_time= 1
        self.e_prev = 0
        

        
        

    def laser_data_callback(self, msg: LaserScan):
        msg.range_max = float('inf')
        cmd = AckermannDriveStamped()
        self.b = msg.ranges[359] #second ray -> theta = 45 degree 
        self.a = msg.ranges[539] # first ray -> angle to the right of car x_axis = 90 
        self.theta = 0.004351851996034384*180  # =45 degrees
    
        self.alpha = arctan((self.a*cos(self.theta)-self.b)/self.a * sin(self.theta))
        self.Dt = self.b * cos(self.alpha)
        self.L = 0.2
        self.Dt1 = self.Dt + self.L * sin(self.alpha)
        self.error = 1.2 - self.Dt1
        
        # self.time_prev = 0
        cmd.drive.steering_angle = self.PID(1.5,1.5,self.error)

        if 0<abs(cmd.drive.steering_angle)<10 :
            cmd.drive.speed = 1.5   

        if 10<=abs(cmd.drive.steering_angle)<20 :
            cmd.drive.speed = 1.0

        else :
              cmd.drive.speed = 0.5
        
        self.pub.publish(cmd)
        print(cmd)

        
    def PID(self,Kp, Kd, Error):
        
        
        
        # PID calculations
        
        P = Kp*Error
        
        current_time = self.current_time
        dt = current_time - self.prev_time
        print(dt)
        # integral = integral + Ki*e*(time - time_prev)
        D = Kd*((Error - self.e_prev)/(dt))# calculate manipulated variable - MV
        MV =  P + D
        # update stored data for next iteration
        self.e_prev = Error
        
        self.prev_time = current_time
        self.current_time += 0.004
        return MV
        
        
        


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
     main()