#!/usr/bin/env python3

import rospy
from dynamixel_sdk import *
from airosa_msgs.msg import DynamixelState, SyncPosition
from sensor_msgs.msg import JointState

# Control table address untuk AX-12
ADDR_AX_TORQUE_ENABLE = 24
ADDR_AX_GOAL_POSITION = 30
ADDR_AX_PRESENT_POSITION = 36
ADDR_AX_PRESENT_SPEED = 38

# Data Byte Length untuk AX-12
LEN_AX_GOAL_POSITION = 2
LEN_AX_PRESENT_POSITION = 2
LEN_AX_PRESENT_SPEED = 2

# Protocol version untuk AX-12
PROTOCOL_VERSION = 1.0

# Pengaturan default
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 1023
DXL_MOVING_STATUS_THRESHOLD = 10

class DynamixelController:
    def __init__(self):
        rospy.init_node('airosa_controller', anonymous=True)
        self.mpi = 3.141592653589793

        self.devicename = rospy.get_param("~devicename", "/dev/U2D2")  # Default ke "/dev/ttyUSB0"
        self.baudrate = rospy.get_param("~baudrate", 1000000)  # Default ke 1000000
        self.motor_ids = rospy.get_param("~motor_ids", [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])  # Default ke motor 1-12
        self.motor_names = rospy.get_param("~motor_names", ["FRcoxa", "FRfemur", "FRtibia", "FLcoxa", "FLfemur", "FLtibia", "BRcoxa", "BRfemur", "BRtibia", "BLcoxa", "BLfemur", "BLtibia"])

        self.portHandler = PortHandler(self.devicename)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)

        self.dynamixel_state_pub = rospy.Publisher('dynamixel_state', DynamixelState, queue_size=2)
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=2)

        self.joint_command_sub = rospy.Subscriber('sync_position', SyncPosition, self.goal_position_callback)

        self.command_received = False

        # Buka port dan set baudrate
        if self.portHandler.openPort():
            rospy.loginfo("Succeeded to open the port")
        else:
            rospy.logerr("Failed to open the port")
            rospy.signal_shutdown("Failed to open the port")

        if not self.portHandler.setBaudRate(self.baudrate):
            rospy.logerr("Failed to change the baudrate")
            rospy.signal_shutdown("Failed to change the baudrate")

        # Aktifkan torsi untuk setiap motor
        for motor_id in self.motor_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr("Motor {}: {}".format(motor_id, self.packetHandler.getTxRxResult(dxl_comm_result)))
                rospy.signal_shutdown("Failed to find motors")
                return
            elif dxl_error != 0:
                rospy.logerr("Motor {}: {}".format(motor_id, self.packetHandler.getRxPacketError(dxl_error)))
            else:
                rospy.loginfo("Dynamixel#{} has been successfully connected".format(motor_id))

    def sync_move_motors(self, motor_ids, goal_positions):
        for motor_id, goal_pos in zip(motor_ids, goal_positions):
            param_goal_position = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos)]
            
            dxl_addparam_result = self.groupSyncWrite.addParam(motor_id, param_goal_position)
            if not dxl_addparam_result:
                rospy.logerr("[ID:{:03d}] groupSyncWrite addparam failed with goal pos {}".format(motor_id, goal_pos))
                rospy.signal_shutdown("Failed to add param for motor {}".format(motor_id))
                quit()
        
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Error: {}".format(self.packetHandler.getTxRxResult(dxl_comm_result)))

        self.groupSyncWrite.clearParam()

        self.pub_all_states(motor_ids, goal_positions)

    def disable_torque(self):
        for motor_id in self.motor_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr("Motor {}: {}".format(motor_id, self.packetHandler.getTxRxResult(dxl_comm_result)))
            elif dxl_error != 0:
                rospy.logerr("Motor {}: {}".format(motor_id, self.packetHandler.getRxPacketError(dxl_error)))

    def close(self):
        self.disable_torque()
        self.portHandler.closePort()

    def raw_to_rad(self, raw):
        degrees = (float(raw) / 1024) * 300 - 60
        return (degrees - 90) * (self.mpi / 180)

    def vel_to_rad(self, vel):
        degrees = (float(vel) / 1024) * 300
        return degrees * (self.mpi / 180)

    def pub_dynamixel_states(self):    
        ids = []
        positions = []
        speeds = []
        joint_ids = [10, 11, 12, 4, 5, 6, 1, 2, 3, 7, 8, 9]

        for motor_id in joint_ids:
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id, ADDR_AX_PRESENT_POSITION)
            dxl_present_speed, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id, ADDR_AX_PRESENT_SPEED)

            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr("Motor {}: {}".format(motor_id, self.packetHandler.getTxRxResult(dxl_comm_result)))
            elif dxl_error != 0:
                rospy.logerr("Motor {}: {}".format(motor_id, self.packetHandler.getRxPacketError(dxl_error)))

            # Append motor ID and its corresponding present position
            ids.append(motor_id)
            positions.append(dxl_present_position)
            speeds.append(dxl_present_speed)

        # Create the ROS message and populate with IDs and positions
        dynamixel_states = DynamixelState()

        dynamixel_states.ids = ids
        dynamixel_states.positions = positions
        dynamixel_states.speeds = speeds

        # Publish the message
        self.dynamixel_state_pub.publish(dynamixel_states)

        # Create the ROS message and populate with IDs and positions
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()

        for idx, motor_id in enumerate(joint_ids):
            # Append motor ID and its corresponding present position
            joint_state.name.append(self.motor_names[idx])
            joint_state.position.append(self.raw_to_rad(positions[idx]))
            joint_state.velocity.append(self.vel_to_rad(speeds[idx]))

        self.joint_state_pub.publish(joint_state)

    def control_loop(self):
        if self.command_received:
            return
        
        self.pub_dynamixel_states()

    def pub_all_states(self, ids, positions):          
        # Create the ROS message and populate with IDs and positions
        speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        dynamixel_states = DynamixelState()

        dynamixel_states.ids = ids
        dynamixel_states.positions = positions
        dynamixel_states.speeds = speeds

        # Create the ROS message and populate with IDs and positions
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()

        for idx, motor_id in enumerate(ids):
            # Append motor ID and its corresponding present position
            joint_state.name.append(self.motor_names[idx])
            joint_state.position.append(self.raw_to_rad(positions[idx]))
            joint_state.velocity.append(self.vel_to_rad(speeds[idx]))

        # Publish the message
        self.joint_state_pub.publish(joint_state)
        self.dynamixel_state_pub.publish(dynamixel_states)

    def goal_position_callback(self, msg):
        self.command_received = True
        self.sync_move_motors(msg.ids, msg.positions)

if __name__ == "__main__":

    controller = DynamixelController()
    rate = rospy.Rate(250)  # 250 Hz

    while not rospy.is_shutdown():
        controller.control_loop()
        rate.sleep()

    controller.close()

