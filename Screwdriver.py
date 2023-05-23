import pyfirmata
import time

POW_PIN_NUM = 3
DIR_PIN_NUM = 7
MAX_SPEED = 80

class Screwdriver:
    def __init__(self, port='COM5'):
        self.board = pyfirmata.Arduino(port)
        self.POW_PIN = self.board.get_pin(f'd:{POW_PIN_NUM}:p')
    
    def __del__(self):
        self.POW_PIN.write(0)
        self.board[DIR_PIN_NUM].write(0)
    
    def turn_on(self, direction=1, time_to_run=3):
        if direction != 0 or direction != 1:
            print("ENTER EITHER 0 OR 1 FOR 'direction' ARGUMENT")
            return 
        self.POW_PIN.write(0)
        self.board.digital[DIR_PIN_NUM].write(direction)
        time.sleep(1)
        for i in range(50, MAX_SPEED):
            self.POW_PIN.write(i/255)
            time.sleep(30/1000)
        time.sleep(time_to_run)
        self.POW_PIN.write(0)
