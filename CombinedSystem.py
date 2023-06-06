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
        self.screwdriver = Screwdriver()
        self.kinematics = Kinematics(joints)
    
    # TODO: add functions to run motors separately
    
    def solve_inverse_kinematics(self, des_points_arr, master_arm_angles, Kp=5):
        return self.kinematics.inverse_kinematics(des_points_arr, master_arm_angles, Kp=Kp)

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
        for target in all_targets:
            for i, pose in enumerate(target):
                dyn_des_pos = {ID:self.dynamixels.degree_to_dxl(pose[i+1]) for i,ID in enumerate(self.dynamixels.ID_LIST)}
                # Execut emotion one-by-one to avoid collision and achieve desired trajectory
                if i == 0:
                    print("Move to first point\n")
                    self.dynamixels.move_motor(des_pos_dict=dyn_des_pos)
                    self.tmotor.move_motor(target_pos=pose[0], threshold=tmotor_threshold, Kp_init=t_Kp_init, Kp_max=t_Kp_max, Kd=t_Kd, degrees=degrees, display=display)
                elif i == 1:
                    print("Turn on screwdriver and move to the hole\n")
                    self.tmotor.move_motor(target_pos=pose[0], threshold=tmotor_threshold, Kp_init=t_Kp_init, Kp_max=t_Kp_max, Kd=t_Kd, degrees=degrees, display=display)
                    self.screwdriver.turn_on()
                    self.dynamixels.move_motor(des_pos_dict=dyn_des_pos)
                elif i == 2:
                    print("Turn off screwdriver and move back\n")
                    self.screwdriver.turn_off()                    
                    self.dynamixels.move_motor(des_pos_dict=dyn_des_pos)
                    self.tmotor.move_motor(target_pos=pose[0], threshold=tmotor_threshold, Kp_init=t_Kp_init, Kp_max=t_Kp_max, Kd=t_Kd, degrees=degrees, display=display)

                print("Press any key to continue! (or press ESC to quit!)\n")
                if getch() == chr(0x1b):
                    break

                
q1, q2, q3, q4, q5 = sp.symbols('q1 q2 q3 q4 q5')
joints = [q1, q2, q3, q4, q5]
system_motors = CombinedSystem(joints, dyn_max_speed=25)

# Set initial motor angles
init_tmotor_pos = system_motors.tmotor.get_current_pos()
init_dyn_pos = [system_motors.dynamixels.get_current_pos(ID) for ID in system_motors.dynamixels.ID_LIST]
print(init_dyn_pos)
init_dyn_pos = [system_motors.dynamixels.dxl_to_degree(value) for value in init_dyn_pos]
init_system_pos = [init_tmotor_pos] + init_dyn_pos
print(init_system_pos)
system_motors.kinematics.set_init_angles(init_system_pos)

all_targets = [[[0, 0, 0, 60, 90], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]] # Example motion

des_points_arr = [[-350+33.5, 350, 30, 90], [-350+73.5, 350, 50, 90]]
master_arm_angles = [[np.rad2deg(-0.25204832), np.rad2deg(-2.10241713)+90], 
                     [np.rad2deg(-0.25204832), np.rad2deg(-2.10241713)+90]]
all_targets = system_motors.solve_inverse_kinematics(des_points_arr, master_arm_angles, Kp=2)

# Example
# Note all angles are in degrees
for target in all_targets:
    for pose in target:
        print(pose)
system_motors.move_all_motors(all_targets, tmotor_threshold=5, t_Kp_init=5, t_Kp_max=500, t_Kd=5)
