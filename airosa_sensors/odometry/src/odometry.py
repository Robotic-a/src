#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import PoseStamped

def odom_callback(msg):
    # Membalik tanda x dan y
    msg.pose.pose.position.x = -msg.pose.pose.position.x
    msg.pose.pose.position.y = -msg.pose.pose.position.y

    # Buat Transformasi
    br = tf.TransformBroadcaster()
    br.sendTransform(
        (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
        rospy.Time.now(),
        "base_footprint",  # frame target
        "odom_rf2o"  # frame source
    )

    # Publikasikan kembali ke topik "odom"
    pub.publish(msg)

if __name__ == '__main__':
    try:
        # Inisialisasi ROS node
        rospy.init_node('odom_to_base_footprint')

        # Buat subscriber untuk topik odom_rf2o
        rospy.Subscriber('/odom_rf2o', Odometry, odom_callback)

        # Buat publisher untuk topik odom
        pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Spin ROS untuk menjaga node berjalan
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

