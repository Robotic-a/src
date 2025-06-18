#!/usr/bin/env python3

# sync_control.py

import rospy
from dynamixel_sdk import *
from airosa_msgs.msg import SyncPosition  # Import pesan kustom
import time

# Control table address untuk AX-12
ADDR_AX_TORQUE_ENABLE = 24
ADDR_AX_GOAL_POSITION = 30
ADDR_AX_PRESENT_POSITION = 36

# Data Byte Length untuk AX-12
LEN_AX_GOAL_POSITION = 2
LEN_AX_PRESENT_POSITION = 2

# Protocol version untuk AX-12
PROTOCOL_VERSION = 1.0

# Pengaturan default
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 1023
DXL_MOVING_STATUS_THRESHOLD = 10

class DynamixelController:
    def __init__(self, devicename, baudrate, motor_ids):
        self.portHandler = PortHandler(devicename)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)
        self.motor_ids = motor_ids
        self.baudrate = baudrate

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
        for motor_id in motor_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr("Motor {}: {}".format(motor_id, self.packetHandler.getTxRxResult(dxl_comm_result)))
            elif dxl_error != 0:
                rospy.logerr("Motor {}: {}".format(motor_id, self.packetHandler.getRxPacketError(dxl_error)))
            else:
                rospy.loginfo("Dynamixel#{} has been successfully connected".format(motor_id))

    def move_motors(self, motor_ids, goal_positions):
        for motor_id, goal_pos in zip(motor_ids, goal_positions):

            param_goal_position = [DXL_LOBYTE(goal_pos), DXL_HIBYTE(goal_pos)]
            
            dxl_addparam_result = self.groupSyncWrite.addParam(motor_id, param_goal_position)
            if not dxl_addparam_result:
                rospy.logerr("[ID:{:03d}] groupSyncWrite addparam failed dengan goal pos {}".format(motor_id, goal_pos))
                rospy.signal_shutdown("Failed to add param for motor {}".format(motor_id))
                quit()
        
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Error: {}".format(self.packetHandler.getTxRxResult(dxl_comm_result)))

        self.groupSyncWrite.clearParam()

        time.sleep(0.03)  # Tidur sebentar untuk memberi kesempatan motor bergerak

    def disable_torque(self):
        for motor_id in self.motor_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr("Motor {}: {}".format(motor_id, self.packetHandler.getTxRxResult(dxl_comm_result)))
            elif dxl_error != 0:
                rospy.logerr("Motor {}: {}".format(motor_id, self.packetHandler.getRxPacketError(dxl_error)))

    def close(self):
        self.portHandler.closePort()


def pub_joints(self):
    ids = []
    positions = []

    for motor_id, goal_pos in goal_positions:
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
            self.portHandler, motor_id, ADDR_AX_PRESENT_POSITION
        )

        if dxl_comm_result != COMM_SUCCESS:
            print(f"Motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Motor {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")

        # Append motor ID and its corresponding present position
        ids.append(motor_id)
        positions.append(dxl_present_position)

    # Create the ROS message and populate with IDs and positions
    msg = SyncPosition()
    msg.ids = ids
    msg.positions = positions

    # Publish the message
    pub.publish(msg)


# Callback function untuk menerima pesan dan menggerakkan motor
def goal_position_callback(msg):
    rospy.loginfo("Received goal positions: {}".format(list(zip(msg.ids, msg.positions))))
    controller.move_motors(msg.ids, msg.positions)

if __name__ == "__main__":
    rospy.init_node("dynamixel_controller")

    # Ambil parameter dari parameter server
    devicename = rospy.get_param("~devicename", "/dev/ttyUSB0")  # Default ke "/dev/ttyUSB0"
    baudrate = rospy.get_param("~baudrate", 1000000)  # Default ke 1000000
    motor_ids = rospy.get_param("~motor_ids", [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])  # Default ke motor 1-12

    # Inisialisasi controller
    controller = DynamixelController(devicename, baudrate, motor_ids)

    # Subscriber untuk mendengarkan posisi yang ingin dicapai
    rospy.Subscriber("sync_position", SyncPosition, goal_position_callback)

    rospy.spin()  # Menunggu pesan dan callback terus berjalan
    controller.close()

