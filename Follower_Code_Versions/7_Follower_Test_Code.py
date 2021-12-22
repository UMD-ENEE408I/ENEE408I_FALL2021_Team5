import logging
import asyncio
import platform
import ast
import struct
import csv
import csv


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

Command_Array = list(csv.reader(open('Arrayfile.csv')))
print(Command_Array)
Command_Count = 0; 

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
    global Command_Array
    global Command_Count
             
    userInput = Command_Array[Command_Count]
    
    print(userInput)
    
    if(userInput == STATE_IDLE):
        await client.write_gatt_char(instrCharacteristic, getBinary(STATE_IDLE))
    elif(userInput == STATE_F):
        await client.write_gatt_char(instrCharacteristic, getBinary(STATE_F))
    elif(userInput == STATE_L):
        await client.write_gatt_char(instrCharacteristic, getBinary(STATE_L))
    elif(userInput == STATE_R):
        await client.write_gatt_char(instrCharacteristic, getBinary(STATE_R))
         
    Command_Count = Command_Count + 1;
 

	
async def recvData(client):
	global DIRECTION_DATA, DISTANCE_DATA, RESERVED_DATA 
	print("Waiting for mouse data...")
	#Read value from mouseDataCharacteristic (Read as BigEndian)
	mouseData = await client.read_gatt_char(mouseDataCharacteristic)
	
	print("Received Mouse Data: ")
	


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
					#Check the node bit to see if it hits a node before sending instruction
					intBuf = int.from_bytes(await client.read_gatt_char(mouseDataCharacteristic), "little")
					#print("intBuf = ", hex(intBuf))
					if(intBuf&0x80000000):
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