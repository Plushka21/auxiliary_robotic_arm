from libs.can import CANDevice, CANSocket
    
class Tmotor:
	def __init__(self) -> None:
		self.can_bus = CANSocket()
		self.encoder = CANDevice(can_bus=self.can_bus)
		print("Created can_bus and encoder")
		self.encoder.send_command(7*b'\xFF' + b'\xFC')  # Enable motor mode
		print("Enabled Motor mode")
	
	def __del__(self):
		self.encoder.send_command(7*b'\xFF' + b'\xFD')  # Close motor mode
	
	def set_zero_pos(self):
		self.encoder.send_command(b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE") # Set zero position

	def move_motor(self, des_pos_list, degrees, threshold=5):
		for des_pos in des_pos_list:
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