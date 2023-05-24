import pyfirmata
import time
import os

POW_PIN_NUM = 3
DIR_PIN_NUM = 7

class Screwdriver:
    def __init__(self, port="/dev/ttyACM0", MAX_SPEED=80):
        os.system(f"arduino --upload StandardFirmata/StandardFirmata.ino --port {port}")
        self.board = pyfirmata.Arduino(port)
        self.POW_PIN = self.board.get_pin(f'd:{POW_PIN_NUM}:p')
        self.MAX_SPEED = MAX_SPEED
    
    def __del__(self):
        self.POW_PIN.write(0)
        self.board.digital[DIR_PIN_NUM].write(0)
    
    def turn_on(self, direction=1, time_to_run=3):
        if direction == 0 or direction == 1:
            self.POW_PIN.write(0)
            self.board.digital[DIR_PIN_NUM].write(direction)
            time.sleep(1)
            for i in range(50, self.MAX_SPEED):
                self.POW_PIN.write(i/255)
                time.sleep(30/1000)
            time.sleep(time_to_run)
            self.POW_PIN.write(0)
            self.board.digital[DIR_PIN_NUM].write(0)
        else:
            print("ENTER EITHER 0 OR 1 FOR 'direction' ARGUMENT")
            return

screw = Screwdriver()
screw.turn_on()
time.sleep(2)
screw.turn_on(0)