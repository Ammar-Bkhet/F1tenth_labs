#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from ackermann_msgs.msg import AckermannDriveStamped


class RelayNode(Node):

    def __init__(self):
        super().__init__("relay_Node")
        self.subscriber = self.create_subscription(AckermannDriveStamped,"drive", self.pose_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, "drive_relay", 5)
        self.get_logger().info("Relay Node has started")
        

    def pose_callback(self, msg: AckermannDriveStamped):
        cmd = AckermannDriveStamped()
        
        cmd.drive.speed = msg.drive.speed * 3
        cmd.drive.steering_angle = msg.drive.steering_angle * 3
        

        self.publisher.publish(cmd)
        self.get_logger().info(f'Publishing : {cmd}')

        


        


def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
     main()