import pyfirmata
import time
import os

POW_PIN_NUM = 3
DIR_PIN_NUM = 7
port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_8563231393835120F042-if00"
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

class Screwdriver:
    def __init__(self, MAX_SPEED=20):
        # os.system(f"arduino --upload StandardFirmata/StandardFirmata.ino --port {port}")
        self.board = pyfirmata.Arduino(port)
        self.POW_PIN = self.board.get_pin(f'd:{POW_PIN_NUM}:p')
        self.MAX_SPEED = MAX_SPEED
        self.test_relay()
    
    def __del__(self):
        self.POW_PIN.write(0)
        self.board.digital[DIR_PIN_NUM].write(0)
    
    def test_relay(self):
        print("Press any key to test relays")
        if getch() == chr(0x1b):
            self.__del__()

        print("Test if relays turn on and off simultaneously")
        self.board.digital[DIR_PIN_NUM].write(1)
        time.sleep(2)
        self.board.digital[DIR_PIN_NUM].write(0)

        print("Press ESC if relay do not work in parallel because it may cause short circuit!\nOtherwise, click any key to continue")
        if getch() == chr(0x1b):
            self.__del__()
    
    def turn_on(self, direction=0, time_to_run=None):
        self.turn_off()
        if direction == 0 or direction == 1:
            self.board.digital[DIR_PIN_NUM].write(direction)
            time.sleep(1)
            for i in range(15, self.MAX_SPEED):
                self.POW_PIN.write(i/255)
                time.sleep(30/1000)
            if time_to_run is not None:
                time.sleep(time_to_run)
                self.POW_PIN.write(0)
                self.board.digital[DIR_PIN_NUM].write(0)
        else:
            print("ENTER EITHER 0 OR 1 FOR 'direction' ARGUMENT")
            return
    
    def turn_off(self):
        self.POW_PIN.write(0)
        self.board.digital[DIR_PIN_NUM].write(0)

# screw = Screwdriver()
# screw.turn_on()
# time.sleep(2)
# screw.turn_off()
