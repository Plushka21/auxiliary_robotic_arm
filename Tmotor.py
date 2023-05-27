from libs.can import CANDevice, CANSocket
import os
    
class Tmotor:
	def __init__(self) -> None:
		os.system("sudo slcand -o -c -s8 /dev/serial/by-id/usb-Protofusion_Labs_CANable_8c005eb_https\:__github.com_normaldotcom_cantact-fw.git_003B00405734570620393235-if00 can0")
		os.system("sudo ifconfig can0 up")
		os.system("sudo ifconfig can0")
		self.can_bus = CANSocket()
		self.encoder = CANDevice(can_bus=self.can_bus)
		print("Created can_bus and encoder")
		self.encoder.send_command(7*b'\xFF' + b'\xFC')  # Enable motor mode
		print("Enabled Motor mode")
	
	def __del__(self):
		self.encoder.send_command(7*b'\xFF' + b'\xFD')  # Close motor mode
	
	def set_zero_pos(self):
		self.encoder.send_command(b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE") # Set zero position

	def move_motor(self, des_pos, degrees=True):
		try:
		# pos = int(des_pos[0] + 2 * threshold)
		# while True:
			if degrees:
				pos, vel, cur = self.encoder.send_deg_command(des_pos)
			else:
				pos, vel, cur = self.encoder.send_rad_command(des_pos)
			# print(pos, end='\r', flush=False)
			# if abs(pos - des_pos[0]) < threshold:
			# 	break
			return pos
		# 	idx+=1
		# 	print("Press any key to continue! (or press ESC to quit!)")
		# 	if getch() == chr(0x1b):
		# 		break
		# self.encoder.send_command(7*b'\xFF' + b'\xFD')
		except KeyboardInterrupt:
			print('Disabled by interrupt')
			self.encoder.send_command(7*b'\xFF' + b'\xFD') # Close motor mode
	
	def display_data(self):
		try:
			while True:
				pos, vel, cur = self.encoder.display_data()
				print(pos, end='\r')
		except KeyboardInterrupt:
			print('Disabled by interrupt')
			self.encoder.send_command(7*b'\xFF' + b'\xFD') # Close motor mode


# tmotor = Tmotor()
# des_pos = (0, 0, 20, 5, 0)
# while des_pos:
# 	pos = tmotor.move_motor(des_pos)
# 	print(pos, end='\r')
# 	if abs(pos) < 1:
# 		des_pos = None

# print(pos)
# while abs(pos) > 0.1:
# 	print(pos, end='\r')
# 	pos = tmotor.move_motor((0, 0, 10, 5, 0))
# print(pos)

# tmotor.display_data()