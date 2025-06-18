#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
import tf
import numpy as np

# Initialize global variables for the previous orientation and position
prev_time = None
prev_orientation = None
prev_position = [0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0]

def cmd_vel_callback(msg):
    global velocity
    # Update the velocity based on the cmd_vel message
    velocity[0] = msg.linear.x  # Linear velocity along x
    velocity[1] = msg.linear.y  # Linear velocity along y
    velocity[2] = msg.angular.z  # Angular velocity around z (yaw)

def imu_callback(msg):
    global prev_time, prev_orientation, prev_position, velocity
    
    # Get the current time
    current_time = msg.header.stamp
    
    # If this is the first message, initialize
    if prev_time is None:
        prev_time = current_time
        prev_orientation = msg.orientation
        prev_position = [0.0, 0.0, 0.0]
        return
    
    # Calculate the time delta
    dt = (current_time - prev_time).to_sec()
    
    # Get the current orientation (as a quaternion)
    current_orientation = msg.orientation

    # Convert quaternion to Euler angles using tf
    quaternion = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
    
    try:
        euler_angles = tf.transformations.euler_from_quaternion(quaternion)
    except Exception as e:
        rospy.logwarn("Could not convert quaternion to Euler angles: %s", str(e))
        return
    
    # Use the previous orientation to calculate the change in position (simple integration of accelerometer data)
    delta_position = np.array(velocity) * dt  # Assuming constant velocity (no acceleration)
    
    # Update the position
    prev_position = list(np.array(prev_position) + delta_position)
    
    # Update previous time and orientation
    prev_time = current_time
    prev_orientation = current_orientation
    
    # Create Odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_footprint"  # Change to base_footprint
    
    # Set the position (x, y, z)
    odom_msg.pose.pose.position.x = prev_position[0]
    odom_msg.pose.pose.position.y = prev_position[1]
    odom_msg.pose.pose.position.z = prev_position[2]
    
    # Set the orientation (as a quaternion)
    odom_msg.pose.pose.orientation = current_orientation
    
    # Set velocity (using cmd_vel input)
    odom_msg.twist.twist.linear.x = velocity[0]
    odom_msg.twist.twist.linear.y = velocity[1]
    odom_msg.twist.twist.linear.z = velocity[2]
    
    # Publish the odometry message
    odom_pub.publish(odom_msg)

def imu_to_odom_node():
    # Initialize the ROS node
    rospy.init_node('imu_to_odom', anonymous=True)

    # Subscribe to the IMU data topic
    rospy.Subscriber("/imu/data", Imu, imu_callback)

    # Subscribe to the cmd_vel topic to get velocity commands
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    # Publisher for the odometry message
    global odom_pub
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    imu_to_odom_node()

