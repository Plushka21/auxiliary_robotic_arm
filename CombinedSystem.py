from Dynamixels import Dynamixels
from Tmotor import Tmotor
import os
import time

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
    def __init__(self):
        self.tmotor = Tmotor()
        self.dynamixels = Dynamixels()
    
    def __del__(self):
        self.tmotor.__del__()
        self.dynamixels.__del__()
    
    def move_dynamixels(self, target_positions_arr, threshold=20, sleep_time=3):
        print("Press any key to start motion! (or press ESC to quit!)\n")
        if getch() == chr(0x1b):
            return
        print("\nATTENTION! EXECUTION STARTS IN..")
        for i in range(sleep_time):
            print(f"{sleep_time-i}...")
            time.sleep(1)
        print("START\n")
        for step_n, des_pos_dict in enumerate(target_positions_arr):
            self.dynamixels.move_motor(des_pos_dict=des_pos_dict, threshold=threshold)
            print(f"Step {step_n+1}/{len(target_positions_arr)} complete\n")
            if step_n + 1 == len(target_positions_arr):
                print("\nALL TASKS EXECUTED\n")
                break
            print("Press any key to continue! (or press ESC to quit!)\n")
            if getch() == chr(0x1b):
                break

    def move_tmotor(self, target_positions_arr, threshold=5, sleep_time=3, degrees=True):
        print("Press any key to start motion! (or press ESC to quit!)\n")
        if getch() == chr(0x1b):
            return
        print("\nATTENTION! EXECUTION STARTS IN..")
        for i in range(sleep_time):
            print(f"{sleep_time-i}...")
            time.sleep(1)
        print("START\n")
        for step_n, des_pos in enumerate(target_positions_arr):
            print(f"Move to angle {des_pos[0]}")
            self.tmotor.move_motor(des_pos=des_pos, degrees=degrees, threshold=threshold)
            print(f"Step {step_n+1}/{len(target_positions_arr)} complete\n")
            if step_n + 1 == len(target_positions_arr):
                print("\nALL TASKS EXECUTED\n")
                break
            print("Press any key to continue! (or press ESC to quit!)\n")
            if getch() == chr(0x1b):
                break
    
init_pos = 0
fin_pos = 4094
dynamixel_targets = [{1: init_pos, 2: init_pos, 3: init_pos, 14: init_pos}, \
           {1: fin_pos, 2: fin_pos, 3: fin_pos, 14: fin_pos}, \
            {1: fin_pos//2, 2: fin_pos//2, 3: fin_pos//2, 14: fin_pos//2}]

tmotor_threshold = 10
Kp = 10
Kd = 3
tmotor_targets = [(i, 0, Kp, Kd, 0) for i in range(0, 100, 90)] #[(0, 0, 2, 2, 0), (90, 0, 2, 2, 0)]
system_motors = CombinedSystem()
system_motors.move_dynamixels(dynamixel_targets)
system_motors.move_tmotor(tmotor_targets, threshold=tmotor_threshold)
# system_motors.tmotor.display_data()
# system_motors.tmotor.set_zero_pos()