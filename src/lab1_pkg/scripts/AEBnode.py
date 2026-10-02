#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node 
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import cos


class AEBNode(Node):

    def __init__(self):
        super().__init__("AEB_Node")
        self.subscriber = self.create_subscription(LaserScan,"/scan", self.laser_data_callback, 10)
        self.subscriber = self.create_subscription(Odometry,"/ego_racecar/odom", self.velocity_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, "/drive", 5)
        # self.publisher2 = self.create_publisher(Odometry, "Testtopic2", 5)
        self.get_logger().info("AEB Node has started")
        

    def laser_data_callback(self, msg: LaserScan):
        msg.range_max = float('inf')
        self.ranges = msg.ranges
        
    
        

        # Define the minimum and maximum angles
        min_angle = -2.3499999046325684
        max_angle = 2.3499999046325684

        # Define the step angle
        step_angle = 0.004351851996034384

        # Define the number of angles to generate
        num_angles = int((max_angle - min_angle) / step_angle) + 1

        # Create an empty array to store the angles
        self.angles = np.zeros(num_angles)

        # Use a for loop to generate the angles
        for i in range(num_angles):
        # Calculate the angle by adding the step angle to the minimum angle
            angle = min_angle + i * step_angle
        # Add the angle to the array
            self.angles[i] = angle

        
        
        
        
        

       

    def velocity_callback(self, msg: Odometry):
        cmd2 = AckermannDriveStamped()
        cmd2.drive.speed = 0.0
        self.Vx = msg.twist.twist.linear.x
        # cmd2.twist = msg.twist
        self.min_time = 1.3
        self.stop = False
        self.ITTC = np.zeros(1080)                   #(self.ranges/max(self.Vx * cos(self.angles), 0))

        for i in range(1080):
            ITTC = (np.float64(self.ranges[i])/abs(max((1 * self.Vx * cos(self.angles[i])), 0)))   #Note : -rdot = Vx cos(theta)
            self.ITTC[i] = ITTC
            # print(self.ITTC[i])
        
        for i in range(1080):
            if self.ITTC[i] < self.min_time :
                self.stop = True
                self.pub.publish(cmd2)
                self.get_logger().info('stop')
            

        # if self.stop == True :
        #     self.pub.publish(cmd2)
        #     self.get_logger().info('published')
        # print(self.ITTC)

        


        


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
     main()