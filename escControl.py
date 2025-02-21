import serial
import time

class ArduinoESC:
	def __init__(self, serialIn, motorChar):
		"""
		Initialize the ZMR ESC for standard range calibration.

		Parameters:
		- pin: GPIO pin connected to the ESC signal wire.
		- frequency: PWM frequency, typically 50 Hz for ESCs.
		"""
		self.ser = serialIn
		self.motorChar = motorChar;
        
	def sendCommand(self, speed):
		string = self.motorChar + str(int(speed)) + '\n'
		print(string)
		self.ser.write(string.encode())
		
