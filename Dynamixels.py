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

    def move_motor(self, des_pos_dict):
        self.update_speed(des_pos_dict.values())
        for ID in des_pos_dict:
            des_pos = des_pos_dict[ID]
            # Write Dynamixel goal position
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, ID, GOAL_POSITION_ADDR_2, des_pos)
                
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

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