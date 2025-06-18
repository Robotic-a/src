#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import sys
import select
import termios
import tty
import threading

# Global variables
cmd_pub = None
twist = Twist()
target_yaw_deg = 0.0
lock = threading.Lock()
error = None  # Simpan error terakhir dari IMU

def imu_callback(data):
    global target_yaw_deg, error

    q = data.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    yaw_deg = math.degrees(yaw)
    yaw_deg = (yaw_deg + 180) % 360 - 180

    with lock:
        target = (target_yaw_deg + 180) % 360 - 180

    err = target - yaw_deg
    err = (err + 180) % 360 - 180

    with lock:
        error = err

    rospy.loginfo("Current yaw: %.2f deg, Target yaw: %.2f deg, Error: %.2f deg", yaw_deg, target, err)

def cmd_vel_pub(event):
    global twist, error

    with lock:
        current_error = error

    if current_error is None:
        return  # Belum ada data IMU

    if abs(current_error) < 1.0:
        return  # Tidak publish jika sudah sangat dekat dengan target

    twist = Twist()
    angular_speed = 0.5
    twist.angular.z = angular_speed if current_error > 0 else -angular_speed

    cmd_pub.publish(twist)


def keyboard_loop():
    global target_yaw_deg

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                c = sys.stdin.read(1)
                if c == 'w':
                    with lock:
                        target_yaw_deg += 15
                        if target_yaw_deg > 180:
                            target_yaw_deg -= 360
                    rospy.loginfo("Increase target yaw to: %.2f", target_yaw_deg)
                elif c == 's':
                    with lock:
                        target_yaw_deg -= 15
                        if target_yaw_deg < -180:
                            target_yaw_deg += 360
                    rospy.loginfo("Decrease target yaw to: %.2f", target_yaw_deg)
                elif c == '\x03':  # Ctrl-C
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        rospy.init_node('imu_to_cmdvel_node')

        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/imu/data', Imu, imu_callback, queue_size=1, tcp_nodelay=True)

        target_yaw_deg = rospy.get_param('~target_yaw', 0.0)
        rospy.loginfo("Target yaw: %.2f deg", target_yaw_deg)

        # Start keyboard input thread
        keyboard_thread = threading.Thread(target=keyboard_loop)
        keyboard_thread.daemon = True
        keyboard_thread.start()

        # Start periodic publisher at 10Hz
        rospy.Timer(rospy.Duration(0.1), cmd_vel_pub)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

