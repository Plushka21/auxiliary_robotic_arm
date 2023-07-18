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
    def __init__(self, joints, dyn_max_speed=50):
        self.tmotor = Tmotor()
        self.dynamixels = Dynamixels(max_speed=dyn_max_speed)
        # self.screwdriver = Screwdriver()
        self.kinematics = Kinematics(joints)
    
    # TODO: add functions to run motors separately
    
    def solve_inverse_kinematics(self, des_points_arr, master_arm_angles, Kp=5, max_dist=1):
        return self.kinematics.inverse_kinematics(des_points_arr, master_arm_angles, Kp=Kp, max_dist=1)

    def move_all_motors(self, all_targets, tmotor_threshold=5, t_Kp_init=1, t_Kp_max=5, t_Kd=5, 
                        degrees=True, sleep_time=3, display=True):
        print("Press any key to start motion! (or press ESC to quit!)\n")
        if getch() == chr(0x1b):
            return
        print("\nATTENTION! EXECUTION STARTS IN..")
        for i in range(sleep_time):
            print(f"{sleep_time-i}..")
            time.sleep(1)
        print()
        dxl_all_poses = []
        dxl_all_times = []
        ak_all_poses = []
        ak_all_times = []
        for target in all_targets:
            for i, pose in enumerate(target):
                dyn_des_pos = {ID:self.dynamixels.degree_to_dxl(pose[i+1]) for i,ID in enumerate(self.dynamixels.ID_LIST)}
                # Execute motion one-by-one to avoid collision and achieve desired trajectory
                if i == 0:
                    print("Move to first point\n")
                    ak_pos_list, ak_time_list = self.tmotor.move_motor(target_pos=pose[0], threshold=tmotor_threshold, Kp_init=t_Kp_init, Kp_max=t_Kp_max, Kd=t_Kd, degrees=degrees, display=display)
                    dxl_pos_list, dxl_time_list = self.dynamixels.move_motor(des_pos_dict=dyn_des_pos)
                elif i == 1:
                    print("Turn on screwdriver and move to the hole\n")
                    ak_pos_list, ak_time_list = self.tmotor.move_motor(target_pos=pose[0], threshold=tmotor_threshold, Kp_init=t_Kp_init, Kp_max=t_Kp_max, Kd=t_Kd, degrees=degrees, display=display)
                    # self.screwdriver.turn_on()
                    dxl_pos_list, dxl_time_list = self.dynamixels.move_motor(des_pos_dict=dyn_des_pos)
                elif i == 2:
                    print("Turn off screwdriver and move back\n")
                    # self.screwdriver.turn_off()
                    dxl_pos_list, dxl_time_list = self.dynamixels.move_motor(des_pos_dict=dyn_des_pos)
                    ak_pos_list, ak_time_list = self.tmotor.move_motor(target_pos=pose[0], threshold=tmotor_threshold, Kp_init=t_Kp_init, Kp_max=t_Kp_max, Kd=t_Kd, degrees=degrees, display=display)
                # ak_pos_list, ak_time_list = self.tmotor.move_motor(target_pos=pose[0], threshold=tmotor_threshold, Kp_init=t_Kp_init, Kp_max=t_Kp_max, Kd=t_Kd, degrees=degrees, display=display)
                # dxl_pos_list, dxl_time_list = self.dynamixels.move_motor(des_pos_dict=dyn_des_pos)
                dxl_all_poses.append(dxl_pos_list)
                dxl_all_times.append(dxl_time_list)
                ak_all_poses.append(ak_pos_list)
                ak_all_times.append(ak_time_list)
                print("Press any key to continue! (or press ESC to quit!)\n")
                if getch() == chr(0x1b):
                    break
        return dxl_all_poses, dxl_all_times, ak_all_poses, ak_all_times

    def move_to_zero_pos(self):
        self.dynamixels.move_motor({ID:self.dynamixels.degree_to_dxl(0) for ID in self.dynamixels.ID_LIST})
        time.sleep(5)
        self.tmotor.move_motor(0, threshold=1)
                
q1, q2, q3, q4, q5 = sp.symbols('q1 q2 q3 q4 q5')
joints = [q1, q2, q3, q4, q5]
system_motors = CombinedSystem(joints, dyn_max_speed=25)
system_motors.tmotor.get_current_pos()
# # Set initial motor angles
# init_tmotor_pos = system_motors.tmotor.get_current_pos()
# init_dyn_pos = [system_motors.dynamixels.get_current_pos(ID) for ID in system_motors.dynamixels.ID_LIST]
# print(init_dyn_pos)
# init_dyn_pos = [system_motors.dynamixels.dxl_to_degree(value) for value in init_dyn_pos]
# init_system_pos = [init_tmotor_pos] + init_dyn_pos
# print(init_system_pos)
# system_motors.kinematics.set_init_angles()#init_system_pos)

# all_targets = [[[0, 0, 0, 60, 90], [0, 0, 0, 0, 0], [30, -30, 45, 60, 90]]] # Example motion

# # des_points_arr = [[-600, 0, 10, 0]]#, [-350+73.5, 350, 50, 90]]
# des_points_arr = [[-600, 0, 10, 0]]
# # master_arm_angles = [[np.rad2deg(-0.25204832), np.rad2deg(-2.10241713)+90]]#, 
#                     #  [np.rad2deg(-0.25204832), np.rad2deg(-2.10241713)+90]]
# master_arm_angles = [[np.rad2deg(-0.25204832), np.rad2deg(-2.10241713)+90]]
# # master_arm_angles = [[0, 0]]           
# all_targets,_ = system_motors.solve_inverse_kinematics(des_points_arr, master_arm_angles, Kp=5, max_dist=1)

# # Example
# # Note all angles are in degrees
# for target in all_targets:
#     for pose in target:
#         print(pose)
# dxl_all_poses, dxl_all_times, ak_all_poses, ak_all_times = system_motors.move_all_motors(all_targets, tmotor_threshold=5, t_Kp_init=10, t_Kp_max=500, t_Kd=5)
# print("Moving to zero position")
# system_motors.move_to_zero_pos()
# file = open("all_results_assembled6.txt", "w")
# file.write(f"{dxl_all_poses}\n{dxl_all_times}\n{ak_all_poses}\n{ak_all_times}")
# file.close()