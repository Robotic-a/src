#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf
import math
import sys
import select
import termios
import tty

def create_quaternion_from_euler(roll, pitch, yaw):
    return tf.transformations.quaternion_from_euler(roll, pitch, yaw)

def get_key(timeout=0.1):
    """Membaca satu tombol dari keyboard (non-blocking)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def imu_dummy_publisher():
    rospy.init_node('imu_dummy_publisher', anonymous=True)
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    yaw_deg = 45.0  # Awal yaw
    roll = 0.0
    pitch = 0.0

    rospy.loginfo("Gunakan tombol 'o' untuk menaikkan yaw +15°, 'l' untuk menurunkan yaw -15°")

    while not rospy.is_shutdown():
        key = get_key()
        if key == 'o':
            yaw_deg += 15.0
        elif key == 'l':
            yaw_deg -= 15.0
        elif key == '\x03':  # Ctrl-C
            break

        yaw_rad = math.radians(yaw_deg)
        quat = create_quaternion_from_euler(roll, pitch, yaw_rad)

        imu_msg = Imu()
        imu_msg.orientation = Quaternion(*quat)
        pub.publish(imu_msg)

        rospy.loginfo("Published dummy IMU yaw: %.2f deg", yaw_deg)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_dummy_publisher()
    except rospy.ROSInterruptException:
        pass

