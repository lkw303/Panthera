'''
		OrienBus: A Python library for interfacing oriental motors with modbus using minimalmodbus python driver

		Authors : "Phone Thiha Kyaw, Monisha, Thein Than Tun"
		Edited by: Kai Wen Lum
		Version : "1.0.1"
		Comments: Work in progress
'''

########################################################################################
#
#   NOTES:

#   - Currently library only support profile 2 with default acceleration and deceleartions
#   - Future Versions will have different profiles and error message feedbacks
#
########################################################################################

import minimalmodbus

_WRITE_REGISTER = 125

_WRITE_REGISTER_GROUP_LOWER = 49
_WRITE_REGISTER_GROUP_UPPER = 48

_FEEDBACK_SPEED_REG_LOWER = 207
_FEEDBACK_SPEED_REG_UPPER = 206

_WRITE_REGISTER_SPEED =1157

_FWD_DEC = 10
_REV_DEC = 18

_WRITE_REGISTER_ACC = 1541
_WRITE_REGISTER_DEC = 1669

_ALARM_OVERCURRENT = 34

_READ_OUTPUT_DRIVER_LOWER = 127
_READ_OUTPUT_DRIVER_UPPER = 126

_READ_TORQUE_UPPER = 264
_READ_TORQUE_LOWER = 265

_MIN_RPM = 80
_MAX_RPM = 4000

_MIN_ACC = 150 #15s
_MAX_ACC = 2 # 0.2s

_MIN_DEC = 150 #15s
_MAX_DEC = 2 # 0.

_ALL_DATA_INITIALIZE_LOWER = 399
_ALL_DATA_INITIALIZE_LOWER = 398

speed = 0

class ModBus(object):

	"""
		ModBus class for talking to instruments (slaves).
		Uses the minimalmodbus python library.
	Args:
		* port (str): The serial port name, for example ``/dev/ttyUSB0`` (Linux),
		  ``/dev/tty.usbserial`` (OS X) or ``COM4`` (Windows).
		* slaveaddress (int): Slave address in the range 1 to 247 (use decimal numbers,
		  not hex). Address 0 is for broadcast, and 248-255 are reserved.
	"""

	def __init__(self, _port, _slave_address):

		self._port = _port
		self._slave_address = _slave_address

		self.instrument = minimalmodbus.Instrument(self._port, self._slave_address)
		self.instrument.serial.baudrate = 9600
		self.instrument.serial.bytesize = 8
		self.instrument.serial.parity   = minimalmodbus.serial.PARITY_EVEN
		self.instrument.serial.stopbits = 1
		self.instrument.serial.timeout  = 0.05
		self.instrument.mode = minimalmodbus.MODE_RTU
		self.instrument.clear_buffers_before_each_transaction = True

		print("Successfully Connected to Slave Address {} ...".format(self._slave_address))

	def writeSpeed(self, speed):

		try:

			if (speed >= _MIN_RPM and speed <= _MAX_RPM):
				self.instrument.write_register(_WRITE_REGISTER_SPEED, speed)
				self.instrument.write_register(_WRITE_REGISTER, _FWD_DEC) # run motor forward with default acceleration

			elif (speed <= -_MIN_RPM and speed >= -_MAX_RPM):
				self.instrument.write_register(_WRITE_REGISTER_SPEED, -speed)
				self.instrument.write_register(_WRITE_REGISTER, _REV_DEC) # run motor backward with default acceleration

			else:
				# self.instrument.write_register(_WRITE_REGISTER, 2) # use this if wants to stop instant (not recommended)
				self.instrument.write_register(_WRITE_REGISTER, 34) # stop motor with default deceleartion
		except:
			pass


	def writeAcc(self, acc):

		try:
			if (acc >= _MAX_ACC and acc <= _MIN_ACC):
				self.instrument.write_register(_WRITE_REGISTER_ACC, acc)
		except:
			print("acc not in range")

	def writeDec(self, dec):

		try:
			if (dec >= _MAX_DEC and dec <= _MIN_DEC):
				self.instrument.write_register(_WRITE_REGISTER_DEC, dec)
		except:
			print("dec not in range")

	def readSpeed(self):

		global speed
		try:
			speed = self.instrument.read_register(_FEEDBACK_SPEED_REG_LOWER) - self.instrument.read_register(_FEEDBACK_SPEED_REG_UPPER)
		except:
			print("Failed to read from instrument")

		return speed

	def read_overcurrent_alarm(self):
		try:
			alarm_status = self.instrument.read_register(_ALARM_OVERCURRENT)
			return alarm_status
		except:
			print("alarm status error")

		
	def read_driver_output(self):
		try:
			driver_output = self.instrument.read_register(_READ_OUTPUT_DRIVER_LOWER)
			return driver_output
		except:
			print("driver output error")

	def read_torque(self):
		try:
			torque = (self.instrument.read_register(_READ_TORQUE_LOWER), self.instrument.read_register(_READ_TORQUE_UPPER))
			return torque
		except:
			print("torque error")

	def reset(self):
		try:
			self.instrument.write_register(_ALL_DATA_INITIALIZE_UPPER,1)
			self.instrument.write_register(_ALL_DATA_INITIALIZE_UPPER,1)
			print("sent reset")
		except:
			print("failed to reset")
	
	def read_reset(self):
		try:
			res_up =self.instrument.read_register(_ALL_DATA_INITIALIZE_UPPER)
			res_low = self.instrument.read_register(_ALL_DATA_INITIALIZE_UPPER)
			print("res up: " + str(res_up))
			print("res low: " + str(res_low))
		except:
			print("failed to read reset")
	
class OrienBus(object):

	def __init__(self, _port):
		self._port = _port

		print("Connecting to port {} ...".format(self._port))

	def initialize(self, _slave_address):
		return ModBus(self._port, _slave_address)
