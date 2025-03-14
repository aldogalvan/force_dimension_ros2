#!/usr/bin/python3

""" A simple example that illustrates use of the Force Dimension ROS2 package.
"""

# ROS imports.
import math
import rclpy
import rclpy.node
from example_interfaces.msg import Float64
from geometry_msgs.msg import Point, Quaternion, TwistStamped, Vector3, PoseStamped
from sensor_msgs.msg import JointState

import numpy as np


# Entry point for ROS2.
def main():
    """ A simple function for creating a ROS2 node that subscribes to Force 
        Dimension position and orientation messages and publishes to /twist_cmds.
    """
    
    # Initialize ROS.
    rclpy.init()
    
    # Create a new ROS2 node.
    node = rclpy.node.Node('optimo_node')
    
    # Initialize variables to hold position and orientation.
    position = None
    orientation = None
    velocity = None
    angular_velocity = None
    gripper_angle = 0
    joint_states = None

    # Define callback functions for the subscribed topics.
    def position_callback(msg):
        nonlocal position
        position = msg

    def orientation_callback(msg):
        nonlocal orientation
        orientation = msg

    def velocity_callback(msg):
        nonlocal velocity
        velocity = norm(msg)

    def angular_velocity_callback(msg):
        nonlocal angular_velocity
        angular_velocity = norm(msg)

    def gripper_angle_callback(msg):
        nonlocal gripper_angle
        gripper_angle = msg.data

    def joint_states_callback(msg):
        nonlocal joint_states
        joint_states = msg
        joint_positions = []
        for position in msg.position:
            joint_positions.append(position)
        node.get_logger().info(f'All joint positions: {joint_positions}')

    # Subscribe to position and orientation topics.
    position_subscription = node.create_subscription(Point, '/robot/feedback/position', position_callback, 10)
    orientation_subscription = node.create_subscription(Quaternion, '/robot/feedback/orientation', orientation_callback, 10)
    velocity_subscription = node.create_subscription(Vector3, '/robot/feedback/velocity', velocity_callback, 10)
    angular_velocity_subscription = node.create_subscription(Vector3, '/robot/feedback/angular_velocity', angular_velocity_callback, 10)
    gripper_angle_subscription = node.create_subscription(Float64, '/robot/feedback/gripper_angle', gripper_angle_callback, 10)
    joint_states_subscription = node.create_subscription(JointState, '/joint_states', joint_states_callback, 10)
    
    # Create a publisher for the force command and twist command
    force_publisher = node.create_publisher(Vector3, '/robot/command/force', 10)
    twist_publisher = node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
    pose_publiser = node.create_publisher(PoseStamped,'/servo_node/pose_cmds',10)
    
    # Define a function to command a force.
    def command_force():
        force_msg = Vector3()
        force_msg.x = 0.0
        force_msg.y = 0.0
        force_msg.z = 0.0
        force_publisher.publish(force_msg)

    # Function placeholder for publishing twist
    def publish_twist():
    	twist_msg = TwistStamped()
    	# help me fill this part
        twist_publisher.publish(twist_msg)
        
    # Function placeholder for publishing twist
    def publish_pose():
    	pose_msg = PoseStamped()
    	# help me fill this part
        pose_publisher.publish(_msg)
        
	

    # Create a timer to publish TwistStamped messages and command a force.
    timer = node.create_timer(0.01, lambda: (publish_twist(), command_force()))
    
    # Spin the node.
    rclpy.spin(node)
    
    # Shutdown ROS.
    rclpy.shutdown()

if __name__ == '__main__':
    main()

