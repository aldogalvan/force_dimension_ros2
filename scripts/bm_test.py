#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TwistStampedPublisher(Node):
    def __init__(self):
        super().__init__('twist_stamped_publisher')
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link7_passive'
        msg.twist.linear.x = 1.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Publishing TwistStamped message')

def main(args=None):
    rclpy.init(args=args)
    
    
    twist_stamped_publisher = TwistStampedPublisher()
    rclpy.spin(twist_stamped_publisher)
    twist_stamped_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

