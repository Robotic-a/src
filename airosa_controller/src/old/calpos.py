#!/usr/bin/env python3

# calpos.py

import math
import numpy as np

class LegMovement:
    def __init__(self, bodyL, C, F, T, offset_leg, servo_pos, coor):
        # Initialize parameters from arguments
        self.bodyL = bodyL
        self.C = C
        self.F = F
        self.T = T
        self.offset_leg = offset_leg
        self.servo_pos = np.array(servo_pos)
        self.coordinates = np.array(coor)

    def calIK(self, x, y, z):
        # Array to store Oc, Of, Ot
        results = [0.0, 0.0, 0.0]

        # Calculate r and a
        r = math.sqrt((x * x) + (y * y))  # r = sqrt(x^2 + y^2)
        a = math.sqrt((z * z) + (r - self.C) * (r - self.C))  # a = sqrt(z^2 + (r - C)^2)

        # Calculate angles f1 and f2
        if (r - self.C) == 0:
            f1 = math.pi / 2 if z > 0 else -math.pi / 2
        else:
            f1 = math.atan(z / (r - self.C))

        # Avoid out-of-bounds error for math.acos
        acos_arg = ((self.F * self.F) + (a * a) - (self.T * self.T)) / (2 * a * self.F)
        acos_arg = max(min(acos_arg, 1), -1)  # Clamping to the range [-1, 1]
        f2 = math.acos(acos_arg)

        # Calculate angles Oc, Of, Ot
        Oc = math.atan2(y, x)  # Oc = atan2(y / x)
        Of = f1 + f2  # Of = f1 + f2

        # Calculate Ot with clamping for acos
        acos_arg_Ot = ((self.F * self.F) + (self.T * self.T) - (a * a)) / (2 * self.F * self.T)
        acos_arg_Ot = max(min(acos_arg_Ot, 1), -1)  # Clamping to [-1, 1]
        Ot = math.acos(acos_arg_Ot)

        # Convert angles from radians to degrees
        Oc = Oc * (180.0 / math.pi)
        Of = Of * (180.0 / math.pi)
        Ot = Ot * (180.0 / math.pi) - 90  # Adjust Ot by -90 degrees

        results[0] = Oc
        results[1] = Of
        results[2] = Ot

        return results

    def gait_trajectory_optimized(self, X0, Y0, Z0, X1, Y1, Z1, isDynamic):
        # Precompute fixed points for X and Y
        Xp1 = Xp2 = X0
        Yp1 = Yp2 = Y0
        Xp3 = Xp4 = X1
        Yp3 = Yp4 = Y1

        if isDynamic:
            Zp1 = Zp4 = Z0
            Zp = 0.05 * Z0 + 0.95 * Z1
            Zp2 = Zp3 = Zp
        else:
            Zp1 = Zp2 = Z0
            Zp3 = Zp4 = Z1

        # List to store the calculated positions
        angles = []

        # Precompute trajectory coefficients (a, b, c, d)
        if isDynamic:
            for t_step in np.linspace(0, 1, 6):  # Generate 6 time steps
                a = (1 - t_step) ** 3
                b = 3 * t_step * (1 - t_step) ** 2
                c = 3 * t_step ** 2 * (1 - t_step)
                d = t_step ** 3

                Px = a * Xp1 + b * Xp2 + c * Xp3 + d * Xp4
                Py = a * Yp1 + b * Yp2 + c * Yp3 + d * Yp4
                Pz = a * Zp1 + b * Zp2 + c * Zp3 + d * Zp4

                # Assuming calIK returns angles for each step
                result = self.calIK(Px, Py, Pz)
                Oc, Of, Ot = result
                angles.append((round(Oc, 2), round(Of, 2), round(Ot, 2)))

        else:
            for t_step in np.linspace(0, 1, 2):  # Generate 1 time step
                a = (1 - t_step) ** 3
                b = 3 * t_step * (1 - t_step) ** 2
                c = 3 * t_step ** 2 * (1 - t_step)
                d = t_step ** 3

                Px = a * Xp1 + b * Xp2 + c * Xp3 + d * Xp4
                Py = a * Yp1 + b * Yp2 + c * Yp3 + d * Yp4
                Pz = a * Zp1 + b * Zp2 + c * Zp3 + d * Zp4

                # Assuming calIK returns angles for each step
                result = self.calIK(Px, Py, Pz)
                Oc, Of, Ot = result
                angles.append((round(Oc, 3), round(Of, 3), round(Ot, 3)))

        return angles

    def rotate_coordinates(self, roll, pitch, yaw, coor_offset):
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        # Rotation Matrices for Roll, Pitch, Yaw
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll_rad), -math.sin(roll_rad)],
            [0, math.sin(roll_rad), math.cos(roll_rad)]
        ])
        Ry = np.array([
            [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
            [0, 1, 0],
            [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]
        ])
        Rz = np.array([
            [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [math.sin(yaw_rad), math.cos(yaw_rad), 0],
            [0, 0, 1]
        ])

        # Apply the rotations in the correct order
        R = np.dot(Rx, np.dot(Ry, Rz))

        rotated_coors = []
        for coor in coor_offset:
            rotated_coor = np.dot(R, coor)
            rotated_coors.append(rotated_coor)

        return rotated_coors, R

    def inDeg(self, angle):
        minAngle = -60
        maxAngle = 240
        angle = max(min(angle, maxAngle), minAngle)  # Clamp the angle to valid range
        return int((angle + 60) / 300 * 1024)

    def calc_multi_leg(self, Xtarget, Ytarget, Ztarget, lift):
        coor = self.coordinates
        coor_offset = coor - self.offset_leg  # Offset for all legs once
        target_positions = coor + np.array([[Xtarget, Ytarget, Ztarget]] * 4) - self.offset_leg

        positions = []
        for i in range(4):
            pos_D = self.gait_trajectory_optimized(coor_offset[i][0], coor_offset[i][1], coor_offset[i][2],
                                                   target_positions[i][0], target_positions[i][1],
                                                   target_positions[i][2] + lift, True)
            pos_S = self.gait_trajectory_optimized(coor_offset[i][0], coor_offset[i][1], coor_offset[i][2],
                                                   target_positions[i][0], target_positions[i][1],
                                                   target_positions[i][2] - lift, True)

            # Clean the position data before adding to positions
            positions.append({
                "D": self.clean_data(pos_D),
                "S": self.clean_data(pos_S)
            })

        allD, allS = [], []
        for step in range(6):
            step_d, step_s = [], []
            for leg in range(4):
                step_d.extend([self.inDeg(self.servo_pos[leg][0] - positions[leg]["D"][step][0]),
                               self.inDeg(self.servo_pos[leg][1] + positions[leg]["D"][step][1]),
                               self.inDeg(self.servo_pos[leg][2] - positions[leg]["D"][step][2])])

                step_s.extend([self.inDeg(self.servo_pos[leg][0] - positions[leg]["S"][step][0]),
                               self.inDeg(self.servo_pos[leg][1] + positions[leg]["S"][step][1]),
                               self.inDeg(self.servo_pos[leg][2] - positions[leg]["S"][step][2])])

            allD.append(step_d)
            allS.append(step_s)

        return allD, allS

    def calc_multi_leg_rotate(self, yaw, lift):
        coor = self.coordinates
        coor_offset = coor - self.offset_leg
        rotated_positions, _ = self.rotate_coordinates(0, 0, yaw, coor)  # Rotation using current positions

        rotated_offset = [rotated - offset for rotated, offset in zip(rotated_positions, self.offset_leg)]

        positions = []
        for i in range(4):
            pos_D = self.gait_trajectory_optimized(coor_offset[i][0], coor_offset[i][1], coor_offset[i][2],
                                                   rotated_offset[i][0], rotated_offset[i][1],
                                                   rotated_offset[i][2] + lift, True)
            pos_S = self.gait_trajectory_optimized(coor_offset[i][0], coor_offset[i][1], coor_offset[i][2],
                                                   rotated_offset[i][0], rotated_offset[i][1],
                                                   rotated_offset[i][2] - lift, True)

            # Clean the position data before adding to positions
            positions.append({
                "D": self.clean_data(pos_D),
                "S": self.clean_data(pos_S)
            })

        rotated = [rot - c for rot, c in zip(rotated_positions, coor)]

        rotD, rotS = [], []
        for step in range(6):
            step_d, step_s = [], []
            for leg in range(4):
                step_d.extend([self.inDeg(self.servo_pos[leg][0] - positions[leg]["D"][step][0]),
                               self.inDeg(self.servo_pos[leg][1] + positions[leg]["D"][step][1]),
                               self.inDeg(self.servo_pos[leg][2] - positions[leg]["D"][step][2])])

                step_s.extend([self.inDeg(self.servo_pos[leg][0] - positions[leg]["S"][step][0]),
                               self.inDeg(self.servo_pos[leg][1] + positions[leg]["S"][step][1]),
                               self.inDeg(self.servo_pos[leg][2] - positions[leg]["S"][step][2])])

            rotD.append(step_d)
            rotS.append(step_s)

        return rotD, rotS, rotated

    def clean_data(self, data):
        tolerance = 1e-14
        return [[0 if abs(x) < tolerance else x for x in row] for row in data]

