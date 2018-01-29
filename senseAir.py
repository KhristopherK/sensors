#!/usr/bin/env python
# -*- coding: utf-8 -*- 
# 
# Khris Kabbabe - The University of Manchester(2017)
# Aerospace Systems Research Group
# 
# This library uses the API provided by SenseAir to use with HPP sensors
# Only tested on HPP_CO2 and HPP_CH4
# Follow the instructions provided for connection and initialisation


# IMPORT external libraries
import serial 	# to talk to serial devices
import time 	# to be able us to pause the code
import datetime	# to read current date and time

# ------------API FROM EM_LoggingHPP_Rev_01.01.pdf------------------#

# FORMAT 				= DEV ID + CMD + LENGHT + CRC16(MODBUS)
ErrorStatus				= 'FE44'+'001E'+'08'+'4211'
ErrorStatus2 			= 'FE44'+'001C'+'08'+'2210'
LPL_flt_ConcPC 			= 'FE44'+'041E'+'10'+'5089'
LPL_uflt_ConcPC 		= 'FE44'+'040E'+'10'+'495D'
LPLDet_Temp 			= 'FE44'+'01A3'+'10'+'D831'
AducDie_Temp 			= 'FE44'+'01B3'+'10'+'183C'
LPL_uflt_Pressure 		= 'FE44'+'0420'+'10'+'E940'
LPL_flt_Pressure 		= 'FE44'+'0422'+'10'+'8941'
Twelve_voltage_mV		= 'FE44'+'02C0'+'10'+'28E9'
VBB_voltage_mV 			= 'FE44'+'02C2'+'10'+'48E8'
NTC0_se_Temp 			= 'FE44'+'0210'+'10'+'E8B4'
NTC1_se_Temp			= 'FE44'+'0212'+'10'+'88B5'
NTC2_se_Temp			= 'FE44'+'0214'+'10'+'28B6'
NTC3_se_Temp			= 'FE44'+'0216'+'10'+'48B7'
NTC4_se_Temp			= 'FE44'+'0218'+'10'+'28B3'
NTC5_se_Temp			= 'FE44'+'021A'+'10'+'48B2'
NTC6_se_Temp			= 'FE44'+'021C'+'10'+'E8B1'
NTC7_se_Temp			= 'FE44'+'021E'+'10'+'88B0'
NTC0_diff_Temp			= 'FE44'+'0230'+'10'+'28AD'
NTC1_diff_Temp			= 'FE44'+'0232'+'10'+'48AC'
NTC2_diff_Temp			= 'FE44'+'0234'+'10'+'E8AF'
NTC3_diff_Temp			= 'FE44'+'0236'+'10'+'88AE'
NTC4_diff_Temp			= 'FE44'+'0238'+'10'+'E8AA'
NTC5_diff_Temp			= 'FE44'+'023A'+'10'+'88AB'
LPL_uflt_IR_Signal		= 'FE44'+'0404'+'10'+'E95B'
LPL_uflt_DT				= 'FE44'+'0406'+'10'+'895A'
LPL_uflt_SclRaw_Signal 	= 'FE44'+'0400'+'20'+'3D59'


# -----------------------SERIAL DEVICE------------------------------#
def serialGeneric(device, baudRate):
	ser = serial.Serial(
	port=device,
	baudrate=baudRate,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS
	#timeout=0
	)
	return ser

# Serial device (port, baudrate)
ser = serialGeneric("/dev/ttyUSB0", 115200)

# Example to read filtered and processed data from sensor

# Function which asks the sensor for data, processes it and gives it to you in PPM for CO2
def getCO2(ser):
	CMD = LPL_flt_ConcPC.decode('hex') 			# From API and decode it as HEX
	ser.write(CMD)								# Sends it to the sensor via serial
	time.sleep(0.15) 							# Wait for data, depends on how fast the sensor reacts, I just put 0.15 sec at ramdom
	co2_hex = (ser.read(21)).encode('hex')		# reads co2 message from sensor, see HPP guide for format. 21 is the number of hex values we are expecing see HPP guide for reference
	co2_dec = int(co2_hex[6:10], 16) / 10.0		# filters the hex string to only the co2 values. Again from HPP guide on reading RAM 
	return co2_dec

# Main part of the code
while True:								# Run until i say othersie
	c_time = datetime.datetime.now() 	# read current time
	co2 = getCO2(ser)					# get co2 value from sensor
	print c_time,'---', co2,' PPM'		# show me the value
	time.sleep (0.5)					# pause for 0.5 seconds

# --- REPEAT --- #


