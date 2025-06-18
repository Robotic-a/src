#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def odom_callback(msg):
    # Membuat objek TransformBroadcaster
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # Ambil timestamp dan set frame
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"           # Frame induk: odom
    t.child_frame_id = "base_footprint"  # Frame anak: base_footprint (misalnya, frame robot)

    # Set posisi dan orientasi dari odometri
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation

    # Publikasikan transformasi
    br.sendTransform(t)

def odom_listener():
    # Inisialisasi node
    rospy.init_node('tf_broadcaster')

    # Subscribing ke topik odom
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # Loop untuk menunggu pesan
    rospy.spin()

if __name__ == '__main__':
    odom_listener()

