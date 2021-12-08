import logging
import asyncio
import platform
import ast
import struct

from bleak import BleakClient
from bleak import BleakScanner
from bleak import discover

# These values have been randomly generated - they must match between the Central and Peripheral devices
# Any changes you make here must be suitably made in the Arduino program as well

instrCharacteristic = '13012F01-F8C3-4F4A-A8F4-15CD926DA146'		#Instructions to send to mouse
mouseDataCharacteristic = '13012F02-F8C3-4F4A-A8F4-15CD926DA146'	#Data received from mouse

STATE_IDLE = 0
STATE_F = 1
STATE_L = 2
STATE_R = 3
STATE_U = 4
STATE_END = 5
STATE_CAL = 6

RED = False


def getBinary(val):			#convert int to its binary value
	if val == STATE_IDLE:
		return bytearray([0x00])
	elif val == STATE_F:
		return bytearray([0x01])
	elif val == STATE_L:
		return bytearray([0x02])
	elif val == STATE_R:
		return bytearray([0x03])
	elif val == STATE_U:
		return bytearray([0x04])
	elif val == STATE_END:
		return bytearray([0x05])
	elif val == STATE_CAL:
		return bytearray([0x06])
	else:
		return bytearray([0x00])

async def sendData(client):
	userInput = input('Enter Instruction to Send :')
	
	#Checking if user input is valid
	while(userInput.isnumeric() == False):
		print("Input must be a numerical from 0-6.")
		userInput = input('Enter Instruction to Send :')
		
	userInput = int(userInput)
	#print(userInput)
    
	if(userInput == STATE_IDLE):
		await client.write_gatt_char(instrCharacteristic, getBinary(STATE_IDLE))
	elif(userInput == STATE_F):
		await client.write_gatt_char(instrCharacteristic, getBinary(STATE_F))
	elif(userInput == STATE_L):
		await client.write_gatt_char(instrCharacteristic, getBinary(STATE_L))
	elif(userInput == STATE_R):
		await client.write_gatt_char(instrCharacteristic, getBinary(STATE_R))
	elif(userInput == STATE_U):
		await client.write_gatt_char(instrCharacteristic, getBinary(STATE_U))
	elif(userInput == STATE_END):
		await client.write_gatt_char(instrCharacteristic, getBinary(STATE_END))
	elif(userInput == STATE_END):
		await client.write_gatt_char(instrCharacteristic, getBinary(STATE_CAL))	
	#check to see if the arduino received the right instruction
	#val = await client.read_gatt_char(instrCharacteristic)	
	#print("Val = ", val)

	
async def recvData(client):
	global DIRECTION_DATA, DISTANCE_DATA, RESERVED_DATA 
	print("Waiting for mouse data...")
	#Read value from mouseDataCharacteristic (Read as BigEndian)
	mouseData = await client.read_gatt_char(mouseDataCharacteristic)
	
	#Convert BigEndian to LittleEndian
	intBuf = int.from_bytes(mouseData, "little")
	print(hex(intBuf))
	
	DIRECTION_DATA = intBuf & 0b1111
	DISTANCE_DATA  = (intBuf & 0xFFFF0) >> 4
	RESERVED_DATA  = (intBuf & 0xFFF00000) >> 20
	print("Direction = {:6d}\t| 0x{:01X}\t\t| 0b{:04b}".format(DIRECTION_DATA, DIRECTION_DATA, DIRECTION_DATA))
	print("Distance  = {:6d}\t| 0x{:04X}\t| 0b{:016b}".format(DISTANCE_DATA, DISTANCE_DATA, DISTANCE_DATA))
	print("Reserved  = {:6d}\t| 0x{:02X}\t\t| 0b{:08b}".format(RESERVED_DATA, RESERVED_DATA, RESERVED_DATA))
	


async def run():
	print('Arduino Nano BLE LED Peripheral Central Service')
	print('Looking for Arduino Nano 33 BLE Sense Peripheral Device...')

	found = False
	devices = await discover()
	for d in devices:       
		if 'FOLLOWER' in d.name:
			print('Found Arduino Nano 33 BLE Sense Peripheral')
			found = True
			async with BleakClient(d.address) as client:
				print(f'Connected to {d.address}')

				while True:
						await recvData(client)
						await sendData(client)
	
	if not found:
		print('Could not find Arduino Nano 33 BLE Sense Peripheral')


loop = asyncio.get_event_loop()
try:
	loop.run_until_complete(run())
except KeyboardInterrupt:
	print('\nRceceived Keyboard Interrupt')
finally:
	print('Program finished')