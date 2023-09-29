#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import sys, tty, termios

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()

        key = self.getKey() #get key from keyboard
        
        if key == 'w':
            msg.linear.x = 0.05 # m/s
        
        elif key == 'x':
            msg.linear.x = -0.05 # m/s

        elif key == 's':
            msg.linear.x = 0.0 # m/s
        
        elif key == 'a':
            msg.angular.z = -0.1 # m/s

        elif key == 'd':
            msg.angular.z = 0.1 # m/s

        
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)

    def getKey(self): #get key from keyboard function
        fd = sys.stdin.fileno()
        old_setting = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_setting)
        
        return ch
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()