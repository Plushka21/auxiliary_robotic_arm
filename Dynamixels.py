#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Protocol Combined Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 1.0 and 2.0
# This example is tested with a Dynamixel MX-28, a Dynamixel PRO 54-200 and an USB2DYNAMIXEL
# Be sure that properties of Dynamixel MX and PRO are already set as %% MX - ID : 1 / Baudnum : 34 (Baudrate : 57600) , PRO - ID : 1 / Baudnum : 1 (Baudrate : 57600)
#

# Be aware that:
# This example configures two different control tables (especially, if it uses Dynamixel and Dynamixel PRO). It may modify critical Dynamixel parameter on the control table, if Dynamixels have wrong ID.
#

import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import numpy as np

if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

TORQUE_ENABLE = 1                 # Value for enabling the torque
TORQUE_DISABLE = 0                 # Value for disabling the torque

TORQUE_ENABLE_ADDR_1 = 24
GOAL_POSITION_ADDR_1 = 30
PRESENT_POSITION_ADDR_1 = 36
PROFILE_SPEED_ADDR_1 = 32

# Control table address for Dynamixel PRO
TORQUE_ENABLE_ADDR_2 = 64
GOAL_POSITION_ADDR_2 = 116
PRESENT_POSITION_ADDR_2 = 132
PROFILE_SPEED_ADDR_2 = 112

# Protocol version
PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel
        
BAUDRATE = 3000000      # Dynamixel default baudrate : 57600

DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4096

class Dynamixels:
    def __init__(self, DEVICENAME = '/dev/ttyUSB0', id_list=[2, 3, 14, 16], max_speed=25) -> None:
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.ID_LIST = id_list
        self.max_speed = max_speed

        self.open_port()
        self.enable_torque()
        self.set_speed()

    def __del__(self):
        # Close port
        self.portHandler.closePort()
    
    def open_port(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
    
    def set_speed(self, id_speed_dict=None):
        if id_speed_dict is None:
            id_speed_dict = {ID: self.max_speed for ID in self.ID_LIST}
        for ID, speed in id_speed_dict.items():
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, ID, PROFILE_SPEED_ADDR_2, speed)

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    # Compute speed for each joint to synchronize joint motion
    def update_speed(self, target_pose):
        cur_dyn_pose = np.array([self.get_current_pos(ID) for ID in self.ID_LIST])
        target_pose = np.array(list(target_pose))

        angle_paths = abs(target_pose - cur_dyn_pose)
        max_angle = max(angle_paths)
        max_time = max_angle / self.max_speed

        upd_speed = {ID: int(path / max_time) for ID, path in zip(self.ID_LIST, angle_paths)}
        self.set_speed(upd_speed)
    
    def enable_torque(self):
        for ID in self.ID_LIST:
            # Enable Dynamixel Torque
            ADDR_MX_TORQUE_ENABLE = TORQUE_ENABLE_ADDR_2

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % ID)
    
    # Convert degree angle to encoder units
    def degree_to_dxl(self, value):
        max_pos, max_deg = 4096, 360.
        pos = int(round((max_pos - 1) * (float(value) / max_deg), 0)) + (max_pos - 1)
        return pos
    
    # Convert encoder units to degree angle
    def dxl_to_degree(self, value):
        max_pos, max_deg = 4096, 360.
        return round(((max_deg * float(value - max_pos - 1)) / (max_pos - 1)), 2)
    
    # Read data from encoder to get the current position
    def get_current_pos(self, ID):
        dxl_present_position, dxl_comm_result, dxl_error = \
                self.packetHandler.read4ByteTxRx(
                    self.portHandler, ID, PRESENT_POSITION_ADDR_2)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Return position in dynamixel scale
        return dxl_present_position

    # def move_motor(self, des_pos_dict):
    #     self.update_speed(des_pos_dict.values())
    #     for ID in des_pos_dict:
    #         des_pos = des_pos_dict[ID]
    #         # Write Dynamixel goal position
    #         dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
    #                 self.portHandler, ID, GOAL_POSITION_ADDR_2, des_pos)
                
    #         if dxl_comm_result != COMM_SUCCESS:
    #             print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
    #         elif dxl_error != 0:
    #             print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def move_motor(self, des_pos_dict):
        self.update_speed(des_pos_dict.values())
        pos_dict_all = {ID: [] for ID in des_pos_dict}
        full_time_dict = {ID: [0] for ID in des_pos_dict}
        for ID in des_pos_dict:
            des_pos = des_pos_dict[ID]
            # Write Dynamixel goal position
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, ID, GOAL_POSITION_ADDR_2, des_pos)
            
            cur_pos = self.get_current_pos(ID)
            pos_dict_all[ID].append(self.dxl_to_degree(cur_pos))
            start_time = time.time()
            dt = 0.01
            while abs(des_pos - cur_pos) > 5:
                # full_time_dict[ID].append(full_time_dict[ID][-1] + dt)
                # time_arr.append(time_arr[-1]+dt)
                cur_pos = self.get_current_pos(ID)
                pos_dict_all[ID].append(self.dxl_to_degree(cur_pos))
            full_time_dict[ID] = [time.time() - start_time]
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return pos_dict_all, full_time_dict

# dyn = Dynamixels()

# angles = [0]
# targets = [dyn.degree_to_dxl(a) for a in angles]
# print(targets)
# print()
# for t in targets:
#     # print(f"Moving to {t}\n")
#     des_pos_dict = {2:t, 3:t, 14:t, 16:t}
#     dyn.move_motor(des_pos_dict)
#     time.sleep(2)
# print([dyn.get_current_pos(ID) for ID in dyn.ID_LIST])

# dyn = Dynamixels()
# angles = [0,0,0,0]
# # angles = [30, 60, -30, 45]
# targets = [dyn.degree_to_dxl(a) for a in angles]
# # dyn.max_speed = 10
# print(targets)
# # print()
# # for t in targets:
# # print(f"Moving to {t}\n")
# des_pos_dict = {2:targets[0], 3:targets[1], 14:targets[2], 16:targets[3]}
# result, full_time_dict = dyn.move_motor(des_pos_dict)
# for d_id in dyn.ID_LIST:
#     print(full_time_dict[d_id])
    # print(dyn.get_current_pos(d_id))
    # time.sleep(2)
# print([dyn.get_current_pos(ID) for ID in dyn.ID_LIST])
# print(result)

# import matplotlib.pyplot as plt

# upd_result = {}
# for ID, old_pos in result.items():
#     upd_pos = [dyn.dxl_to_degree(p) for p in old_pos]
#     upd_result[ID] = upd_pos
# fig, ax = plt.subplots()
# ax.plot(upd_result[2])
# ax.plot(upd_result[3])
# ax.plot(upd_result[14])
# ax.plot(upd_result[16])

# # fig.savefig("dxl_result.png")

# file = open('dxl_result.txt', 'w')
# for res in upd_result.items():
#     file.write(f"{res}\n")
# for time_res in full_time_dict.items():
#     file.write(f"{time_res}\n")
# file.close()