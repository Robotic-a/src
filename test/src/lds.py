#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

# Threshold jarak minimum dan maksimum untuk berhenti (dalam meter)
distance_lower_threshold = 0.15  # 15 cm
distance_upper_threshold = 0.20  # 20 cm

# Sudut yang ingin ditampilkan (dalam derajat)
target_angles_deg = [0, 45, 90, 135, 180, 225, 270, 315]

# Deteksi obstacle hanya di sudut 0°
check_zone_deg = [0]

# Global publisher harus di luar fungsi
cmd_pub = None

def callback(scan):
    global cmd_pub
    angle_min = scan.angle_min
    angle_increment = scan.angle_increment
    ranges = scan.ranges

    obstacle_detected = False
    for angle_deg in check_zone_deg:
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - angle_min) / angle_increment)
        if 0 <= index < len(ranges):
            distance = ranges[index]
            if not math.isinf(distance) and distance_lower_threshold <= distance <= distance_upper_threshold:
                obstacle_detected = True
                break

    cmd = Twist()
    if obstacle_detected:
        rospy.logwarn(f"Obstacle detected within {distance_lower_threshold*100:.0f}-{distance_upper_threshold*100:.0f} cm at 0°! Stopping robot.")
        cmd.linear.x = 0.0
    else:
        cmd.linear.x = -1.0  # kecepatan normal

    cmd_pub.publish(cmd)

    # Tampilkan data sudut tertentu
    rospy.loginfo("Filtered LiDAR data:")
    for angle_deg in target_angles_deg:
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - angle_min) / angle_increment)
        if 0 <= index < len(ranges):
            distance = ranges[index]
            rospy.loginfo(f"Angle {angle_deg}°: {distance:.2f} m")

def listener():
    global cmd_pub
    rospy.init_node('filtered_lidar_with_stop', anonymous=True)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

