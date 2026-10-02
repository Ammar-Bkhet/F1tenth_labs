#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from ackermann_msgs.msg import AckermannDriveStamped

class Talker_pub(Node):
    
    def __init__(self):
        super().__init__('talker_pub')
        self.pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.timer = self.create_timer(0.1, self.callback)
        self.i = 0
        

    def callback(self) :
        v = 5.0
        d = 20.0
        msg = AckermannDriveStamped()
        msg.drive.speed = v
        msg.drive.steering_angle = d
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing : {msg}')
        self.i += 1





def main(args = None):
    rclpy.init(args = args)
    node = Talker_pub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()