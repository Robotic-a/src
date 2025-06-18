#!/usr/bin/env python3

import rospy
import time
from airosa_msgs.msg import SyncPosition
from calpos import LegMovement
from geometry_msgs.msg import Twist


# Initialize ROS Node and Publisher
rospy.init_node('dynamixel_operator')

pub = rospy.Publisher('/sync_position', SyncPosition, queue_size=1)

# Global variable for received command
command = None

# Fetch parameters from the parameter server
robot_params = rospy.get_param('robot', {})


# Command move callback to handle movement commands
def command_move_callback(msg):
    global last_z
    lx = msg.linear.x * 5
    ly = msg.linear.y * 5
    lz = msg.linear.z * 5
    ax = msg.angular.x * 2.5
    ay = msg.angular.y * 2.5
    az = msg.angular.z * 2.5

    if (lx != 0 and lz == 0) or (ly != 0 and lz == 0):
        linear(lx,-ly,lz)
    elif lz != 0 and lx == 0 and ly == 0:
        lift(lz)
        print (last_z)
    elif az != 0:
        angular(az)
    else:
        stop()

# Trot gait to calculate positions for legs
def trot_gait(isComb1, isComb2, x, y, z, Zp, dT):
    coor = rospy.get_param('robot/coor', [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    leg = LegMovement(L, C, F, T, offset_leg, servo_pos, coor)

    allD, _ = leg.calc_multi_leg(x, y, z, Zp)
    _, allS = leg.calc_multi_leg(-x, -y, z, Zp)

    for step in range(6):
        group = []
        for i in range(12):
            posD = allD[step][i]
            posS = allS[step][i]
            serv = motor_ids[i]
            if isComb1:
                if i in [0, 1, 2, 9, 10, 11]:
                    group.append((serv, posD))
                else:
                    group.append((serv, posS))
            if isComb2:
                if i in [3, 4, 5, 6, 7, 8]:
                    group.append((serv, posD))
                else:
                    group.append((serv, posS))

        # Publish the motor positions
        publish_pos(group)
        time.sleep(dT)

    # Update coordinates based on combinations
    if isComb1:
        for i in range(4):
            coor[i] = [coord + delta for coord, delta in zip(coor[i], [x, y, z])]
    if isComb2:
        for i in range(4):
            coor[i] = [coord - delta for coord, delta in zip(coor[i], [x, y, z])]

    # Set updated coordinates back to parameter server
    rospy.set_param('robot/coor', coor)

def trot_gait_rotate(isComb1, isComb2, yaw, Zp, dT):
    coor = rospy.get_param('robot/coor', [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    leg = LegMovement(L, C, F, T, offset_leg, servo_pos, coor)

    # Perform the rotations
    rotD, _, rotateD = leg.calc_multi_leg_rotate(yaw, Zp)
    _, rotS, rotateS = leg.calc_multi_leg_rotate(-yaw, Zp)

    for step in range(6):
        group = []
        for i in range(12):
            posD = rotD[step][i]
            posS = rotS[step][i]
            serv = motor_ids[i]

            if isComb1:
                if i in [0, 1, 2, 9, 10, 11]:
                    group.append((serv, posD))
                else:
                    group.append((serv, posS))
            if isComb2:
                if i in [3, 4, 5, 6, 7, 8]:
                    group.append((serv, posD))
                else:
                    group.append((serv, posS))
                    
        publish_pos(group)
        time.sleep(dT)

    # Update the coordinates for each leg
    if isComb1:
        coor[0] = [round(c + x, 6) for c, x in zip(coor[0], rotateD[0])]
        coor[3] = [round(c + x, 6) for c, x in zip(coor[3], rotateD[3])]
        coor[1] = [round(c + x, 6) for c, x in zip(coor[1], rotateS[1])]
        coor[2] = [round(c + x, 6) for c, x in zip(coor[2], rotateS[2])]

    if isComb2:
        coor[0] = [round(c + x, 6) for c, x in zip(coor[0], rotateS[0])]
        coor[3] = [round(c + x, 6) for c, x in zip(coor[3], rotateS[3])]
        coor[1] = [round(c + x, 6) for c, x in zip(coor[1], rotateD[1])]
        coor[2] = [round(c + x, 6) for c, x in zip(coor[2], rotateD[2])]

    # Set updated coordinates back to parameter server
    rospy.set_param('robot/coor', coor)

# Function to publish motor positions
def publish_pos(group):
    ids = []
    positions = []

    for motor_id, goal_pos in group:
        ids.append(motor_id)
        positions.append(goal_pos)

    msg = SyncPosition()
    msg.ids = ids
    msg.positions = positions
    pub.publish(msg)

def linear(stepx,stepy,stepz):
    trot_gait(1, 0, stepy, stepx, stepz, 10, 0.03)
    trot_gait(0, 1, stepy, stepx, stepz, 10, 0.03)

def angular(anglez):
    trot_gait_rotate(1, 0, anglez, 10, 0.03)
    trot_gait_rotate(0, 1, anglez, 10, 0.03)

def stop():
    x = 0
    y = 0
    z = 0
    Zp = 0
    dT = 0.01
    
    coor = rospy.get_param('robot/coor', [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    leg = LegMovement(L, C, F, T, offset_leg, servo_pos, coor)

    allD, _ = leg.calc_multi_leg(x, y, z, Zp)

    for step in range(6):
        group = []
        for i in range(12):
            posD = allD[step][i]
            serv = motor_ids[i]
            group.append((serv, posD))

        # Publish the motor positions
        publish_pos(group)
        time.sleep(dT)

    # Update coordinates based on combinations
    for i in range(4):
        coor[i] = [coord + delta for coord, delta in zip(coor[i], [x, y, z])]

    # Set updated coordinates back to parameter server
    rospy.set_param('robot/coor', coor)

last_z = 0

def reset():
    global last_z
    x = 0
    y = 0
    z = -last_z
    Zp = 0
    dT = 0.01
    
    coor = rospy.get_param('robot/coor', [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    allD, _ = leg.calc_multi_leg(x, y, z, Zp)

    for step in range(6):
        group = []
        for i in range(12):
            posD = allD[step][i]
            serv = motor_ids[i]
            group.append((serv, posD))

        # Publish the motor positions
        publish_pos(group)
        time.sleep(dT)

    # Update coordinates based on combinations
    for i in range(4):
        coor[i] = [coord + delta for coord, delta in zip(coor[i], [x, y, z])]

    last_z = 0
    # Set updated coordinates back to parameter server
    rospy.set_param('robot/coor', coor)

def lift(lz):
    global last_z
    last_z += lz
    x = 0
    y = 0
    z = 0
    Zp = 0
    dT = 0.03
    
    coor = rospy.get_param('robot/coor', [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    # Update coordinates based on combinations
    for i in range(4):
        coor[i] = [coord + delta for coord, delta in zip(coor[i], [x, y, lz])]
     
    # Set updated coordinates back to parameter server
    rospy.set_param('robot/coor', coor)
    coor_new = rospy.get_param('robot/coor', [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    leg = LegMovement(L, C, F, T, offset_leg, servo_pos, coor_new)

    allD, _ = leg.calc_multi_leg(x, y, z, Zp)

    for step in range(6):
        group = []
        for i in range(12):
            posD = allD[step][i]
            serv = motor_ids[i]
            group.append((serv, posD))

        # Publish the motor positions
        publish_pos(group)
        time.sleep(dT)

if __name__ == '__main__':
    try:
        # Robot Parameters
        L = rospy.get_param('/robot/body_parameters/L')
        C = rospy.get_param('/robot/body_parameters/C')
        F = rospy.get_param('/robot/body_parameters/F')
        T = rospy.get_param('/robot/body_parameters/T')
        offset_leg = rospy.get_param('/robot/leg_offset')
        servo_pos = rospy.get_param('/robot/servo_pos')
        coor = rospy.get_param('/robot/coor')

        # Servo IDs
        motor_ids = [10, 11, 12, 4, 5, 6, 1, 2, 3, 7, 8, 9]

        # Initialize LegMovement with parameters
        leg = LegMovement(L, C, F, T, offset_leg, servo_pos, coor)

        # Initial robot movement
        trot_gait(1, 0, 0, 0, 0, 0, 0.3)
        trot_gait(0, 1, 0, 0, 0, 0, 0.3)

        # Subscribe to IMU and command move topics
        rospy.Subscriber("/cmd_vel", Twist, command_move_callback)

        rospy.spin()  # Keep the node running

    except rospy.ROSInterruptException:
        pass

