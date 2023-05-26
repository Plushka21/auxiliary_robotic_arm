from Dynamixels import Dynamixels
from Tmotor import Tmotor
from Screwdriver import Screwdriver
from Kinematics import Kinematics
import os
import time
import numpy as np
import sympy as sp

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

class CombinedSystem:
    def __init__(self, joints):
        self.tmotor = Tmotor()
        self.dynamixels = Dynamixels()
        self.screwdriver = Screwdriver()
        self.kinematics = Kinematics(joints)
    
    def __del__(self):
        self.tmotor.__del__()
        self.dynamixels.__del__()
        self.screwdriver.__del__()
    
    # TODO: fix functions to tun motors separately
    # def move_dynamixels(self, target_positions_arr, threshold=20, sleep_time=3):
    #     print("Press any key to start motion! (or press ESC to quit!)\n")
    #     if getch() == chr(0x1b):
    #         return
    #     print("\nATTENTION! EXECUTION STARTS IN..")
    #     for i in range(sleep_time):
    #         print(f"{sleep_time-i}...")
    #         time.sleep(1)
    #     print("START\n")
    #     for step_n, des_pos_dict in enumerate(target_positions_arr):
    #         reached_ID_list = []
    #         while True:
    #             cur_pos_dict = self.dynamixels.move_motor(des_pos_dict=des_pos_dict, threshold=threshold)
    #             for ID, cur_pos in cur_pos_dict.items():
    #                 if abs(des_pos_dict[ID] - cur_pos) < threshold:

    #         print(f"Step {step_n+1}/{len(target_positions_arr)} complete\n")
    #         if step_n + 1 == len(target_positions_arr):
    #             print("\nALL TASKS EXECUTED\n")
    #             break
    #         print("Press any key to continue! (or press ESC to quit!)\n")
    #         if getch() == chr(0x1b):
    #             break

    # def move_tmotor(self, target_positions_arr, threshold=5, degrees=True, sleep_time=3):
    #     print("Press any key to start motion! (or press ESC to quit!)\n")
    #     if getch() == chr(0x1b):
    #         return
    #     print("\nATTENTION! EXECUTION STARTS IN..")
    #     for i in range(sleep_time):
    #         print(f"{sleep_time-i}...")
    #         time.sleep(1)
    #     print("START\n")
    #     for step_n, des_pos in enumerate(target_positions_arr):
    #         print(f"Move to angle {des_pos[0]}")
    #         self.tmotor.move_motor(des_pos=des_pos, degrees=degrees, threshold=threshold)
    #         print(f"Step {step_n+1}/{len(target_positions_arr)} complete\n")
    #         if step_n + 1 == len(target_positions_arr):
    #             print("\nALL TASKS EXECUTED\n")
    #             break
    #         print("Press any key to continue! (or press ESC to quit!)\n")
    #         if getch() == chr(0x1b):
    #             break
    
    def get_des_positions(self, des_points_arr, master_arm_angles):
        return self.kinematics.inverse_kinematics(des_points_arr, master_arm_angles)
    
    def move_all_motors(self, all_targets, tmotor_threshold=5, t_Kp=5, t_Kd=3, 
                        dyn_threshold=20, degrees=True, sleep_time=3):
        print("Press any key to start motion! (or press ESC to quit!)\n")
        if getch() == chr(0x1b):
            return
        print("\nATTENTION! EXECUTION STARTS IN..")
        for i in range(sleep_time):
            print(f"{sleep_time-i}..")
            time.sleep(1)
        print()
        for target in all_targets:
            for i, pose in enumerate(target):
                print(pose)
                if i == 0:
                    print("Move to first point\n")
                elif i == 1:
                    print("Turn on screwdriver and move to the hole\n")
                    self.screwdriver.turn_on()
                elif i == 2:
                    print("Turn off screwdriver and move back\n")
                    self.screwdriver.turn_off()
                tmotor_des_pos = (pose[0], 0, t_Kp, t_Kd, 0)
                dyn_des_pos = {ID:pose[i+1] for i,ID in enumerate(self.dynamixels.ID_PROT_DICT.keys())}
                while (tmotor_des_pos is not None) or len(dyn_des_pos.keys()) > 0:
                    cur_tmotor_pos = self.tmotor.move_motor(
                        des_pos=tmotor_des_pos, degrees=degrees, threshold=tmotor_threshold)
                    cur_dyn_pos_dict = self.dynamixels.move_motor(
                        des_pos_dict=dyn_des_pos, threshold=dyn_threshold)
                
                    for ID in cur_dyn_pos_dict.keys():
                        if abs(cur_dyn_pos_dict[ID] - dyn_des_pos[ID]) < dyn_threshold:
                            del (dyn_des_pos[ID])
                    if (tmotor_des_pos is not None) and abs(cur_tmotor_pos - tmotor_des_pos[0]) < tmotor_threshold:
                        tmotor_des_pos = None
                
                print("Press any key to continue! (or press ESC to quit!)\n")
                if getch() == chr(0x1b):
                    break
        # for step_n, (tmotor_des_pos, dyn_des_pos) in enumerate(zip(tmotor_target_positions_arr, dyn_target_positions_arr)):
        #     while len(tmotor_des_pos) > 0 or len(dyn_des_pos.keys()) > 0:
        #         cur_tmotor_pos = self.tmotor.move_motor(des_pos_list=tmotor_des_pos, degrees=degrees, threshold=tmotor_threshold)
        #         cur_dyn_pos_dict = self.dynamixels.move_motor(des_pos_dict=dyn_des_pos, threshold=dyn_threshold)
        #         for ID in cur_dyn_pos_dict.keys():
        #             if abs(cur_dyn_pos_dict[ID] - dyn_des_pos[ID]) < dyn_threshold:
        #                 del(dyn_des_pos[ID])
        #         if len(tmotor_des_pos) > 0 and abs(cur_tmotor_pos - tmotor_des_pos[0][0]) < tmotor_threshold:
        #             del(tmotor_des_pos[0])
        #     print(f"Step {step_n+1}/{len(tmotor_target_positions_arr)} complete\n")
        #     if step_n + 1 == len(tmotor_target_positions_arr):
        #         print("\nALL TASKS EXECUTED\n")
        #         break
                

q1, q2, q3, q4, q5 = sp.symbols('q1 q2 q3 q4 q5')
joints = [q1, q2, q3, q4, q5]
system_motors = CombinedSystem(joints)
init_pos = 0
fin_pos = 4094
dynamixel_targets = [{1: init_pos, 2: init_pos, 3: init_pos, 14: init_pos}, \
           {1: fin_pos, 2: fin_pos, 3: fin_pos, 14: fin_pos}]#, \
            #{1: fin_pos//2, 2: fin_pos//2, 3: fin_pos//2, 14: fin_pos//2}]
Kp = 3
Kd = 3
tmotor_targets = [[(i, 0, Kp, Kd, 0)] for i in range(0, 100, 90)] #[(0, 0, 2, 2, 0), (90, 0, 2, 2, 0)]

Kp = 10
Kd = 3
des_points_arr = [
    [-610, 225, 255, np.radians(-90)]]#, [-660, 225, 255, np.radians(-90)]]
master_arm_angles = [
    [np.radians(30), np.radians(-30)]]#, [np.radians(30), np.radians(-30)]]
all_targets = system_motors.get_des_positions(
    des_points_arr, master_arm_angles)
system_motors.move_all_motors(all_targets)
