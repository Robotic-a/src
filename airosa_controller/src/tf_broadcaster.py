#!/usr/bin/env python3
# coding: utf-8

import rospy
import tf2_ros
import geometry_msgs.msg
import tf

def publish_transform():
    # Create a TransformBroadcaster to send out the transform
    br = tf2_ros.TransformBroadcaster()

    # Define the transform
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()  # Timestamp
    t.header.frame_id = "odom"  # Parent frame
    t.child_frame_id = "base_link"  # Child frame
    
    # Define the translation and rotation (Quaternion)
    t.transform.translation.x = 1.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, 1.57)  # Example rotation in radians (90 degrees around Z axis)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # Broadcast the transform
    br.sendTransform(t)

def main():
    try:
        rospy.init_node('tf_publisher_node')
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            publish_transform()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

