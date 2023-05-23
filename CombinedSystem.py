import Dynamixels, Tmotor
import os

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
        self.dynamixels = Dynamixels
    
    def __del__(self):
        self.tmotor.__del__()
        self.dynamixels.__del__()
        while True:
            print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break
    
