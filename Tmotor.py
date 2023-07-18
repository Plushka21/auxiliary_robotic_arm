from libs.can import CANDevice, CANSocket
import os
import numpy as np
import time
    
class Tmotor:
	def __init__(self) -> None:
		# Create CAN connection with CANable
		os.system("sudo slcand -o -c -s8 /dev/serial/by-id/usb-Protofusion_Labs_CANable_8c005eb_https\:__github.com_normaldotcom_cantact-fw.git_003B00405734570620393235-if00 can0")
		os.system("sudo ifconfig can0 up")
		# os.system("sudo ifconfig can0")
		self.can_bus = CANSocket()
		self.encoder = CANDevice(can_bus=self.can_bus)
		print("Created can_bus and encoder")
		self.encoder.send_command(7*b'\xFF' + b'\xFC')  # Enable motor mode
		print("Enabled Motor mode")
	
	def disable_motor_mode(self):
		self.encoder.send_command(7*b'\xFF' + b'\xFD')  # Close motor mode
	
	def set_zero_pos(self):
		self.encoder.send_command(7*b'\xFF' + b'\xFE') # Set the current position as zero position

	def move_motor(self, target_pos, threshold=5, Kp_init=1, Kp_max=500, Kd=5, degrees=True, display=True):
		try:
			t = 1
			Kp_cur = Kp_init
			pos_arr = []
			# dt = 0.01
			start_time = time.time()
			# Move motor until position is in range of threshold from desired positions
			while True:
				des_pos = (target_pos, 0, Kp_cur, Kd, 0)
				if degrees:
					real_pos, vel, cur = self.encoder.send_deg_command(des_pos)
				else:
					real_pos, vel, cur = self.encoder.send_rad_command(des_pos)
				pos_arr.append(real_pos)
				# if not len(time_arr):
				# 	time_arr.append(0)
				# else:
				# 	time_arr.append(time_arr[-1]+dt)
				if display:
					print(f"{round(real_pos, 2)} approaches {target_pos} with threshold {threshold}", end='\r', flush=False)
				t += 1
				if Kp_cur < Kp_max and t % 10 == 0:
					Kp_cur += 0.5
				if abs(real_pos - target_pos) < threshold:
					break
			time_arr = [time.time() - start_time]
			# When motor reached desired pose, fix the motor with maximum Kp value
			fix_pose = (real_pos, 0, self.encoder.motorParams['KP_MAX'], Kd, 0)
			self.encoder.send_deg_command(fix_pose)
			print(f"{round(real_pos, 2)} approached {target_pos} with threshold {threshold}")
			# return target_pos, 
			return pos_arr, time_arr
		except KeyboardInterrupt:
			print('Disabled by interrupt')
			self.encoder.send_command(7*b'\xFF' + b'\xFD') # Close motor mode
	
	# Get current position from encoder
	def get_current_pos(self, degrees=True):
		id, dlc, motorStatusData = self.encoder.reciver()
		rawMotorData = self.encoder.decode_motor_status(motorStatusData)
		pos, vel, curr = self.encoder.convert_raw_to_physical_rad(rawMotorData[0], rawMotorData[1], rawMotorData[2])
		if not degrees:
			pos = np.radians(pos)
		return pos
	
	# Display the position without any resistance from the motor
	def display_data(self, Kd=0):
		while True:
			# pos = self.get_current_pos()
			pos = self.encoder.send_deg_command((0,0,0,Kd,0))[0]
			print(pos, end='\r')
			# self.encoder.send_command(7*b'\xFF' + b'\xFD') # Close motor mode


# time.sleep(3)
# Kp_cur = 1
# Kp_max = 10
tmotor = Tmotor()
tmotor.set_zero_pos()
print(tmotor.get_current_pos())
# time.sleep(2)
# tmotor.disable_motor_mode()
# p = 0
pos_arr, time_arr = tmotor.move_motor(target_pos=0, Kp_init=1, threshold=0.1)
# print(time_arr)
# print(len(pos_arr))
# print(len(time_arr))
# file = open('ak_result.txt', 'w')
# file.write(f"{pos_arr}\n{time_arr}")
# file.close()

# print(pos)
# while abs(pos) > 0.1:
# 	print(pos, end='\r')
# 	pos = tmotor.move_motor((0, 0, 10, 5, 0))
# print(pos)