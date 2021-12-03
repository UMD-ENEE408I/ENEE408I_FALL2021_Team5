# -*- coding: utf-8 -*-
"""
/******************************************************************* 
  Scout Service Central
  ENEE408I Team 5

    Last edited: 11/30/21 5:02PM

  This code receives a byte from the mouse, decodes it, and prints it
 */
"""


import logging
import asyncio
import platform
import ast
import signal

from bleak import BleakClient
from bleak import BleakScanner
from bleak import discover

from enum import Enum

# These values have been randomly generated - they must match between the Central and Peripheral devices
# Any changes you make here must be suitably made in the Arduino program as well

RED_LED_UUID = '13012F01-F8C3-4F4A-A8F4-15CD926DA146'
GREEN_LED_UUID = '13012F02-F8C3-4F4A-A8F4-15CD926DA146'
BLUE_LED_UUID = '13012F03-F8C3-4F4A-A8F4-15CD926DA146'

STATUS_CHAR_UUID = '19d10001-e8f2-537e-4f6c-d104768a1214'
COMMAND_CHAR_UUID = '19d10002-e8f2-537e-4f6c-d104768a1214'
XPOS_CHAR_UUID = '19d10003-e8f2-537e-4f6c-d104768a1214'
YPOS_CHAR_UUID = '19d10004-e8f2-537e-4f6c-d104768a1214'

scout1_init = False
DEBUG_PRINT_STATUS = False # print out the status every time
scout1_curr_command = 0B00000000

on_value = bytearray([0x01])
off_value = bytearray([0x00])

class Command(Enum):
    MOVE_FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    TURN_AROUND = 4
    SET_MODE_SCOUT = 5
    SET_MODE_FOLLOWING = 6



def status_decode(val):
    toReturn = ""
    #check mouse mode
    if (val & 0B10000000 == 0B10000000):
        toReturn += "atExit\t"
    else: toReturn += "!atExit\t"
    if (val & 0B01000000 == 0B01000000):
        toReturn += "atDeadEnd\t"
    else: toReturn += "!atDeadEnd\t"
    if (val & 0B00100000 == 0B00100000):
        toReturn += "isForward\t"
    else: toReturn += "!isForward\t"
    if (val & 0B00010000 == 0B00010000):
        toReturn += "atLeft\t"
    else: toReturn += "!atLeft\t"
    if (val & 0B00001000 == 0B00001000):
        toReturn += "atRight\t"
    else: toReturn += "!atRight\t"
    if (val & 0B00000100 == 0B00000100):
        toReturn += "atIntersection\t"
    else: toReturn += "!atIntersection\t"
    if (val & 0B00000010 == 0B00000010):
        toReturn += "onWhiteLine\t"
    else: toReturn += "!onWhiteLine\t"
    if (val & 0B00000001 == 0B00000001):
        toReturn += "MODE: FOLLOWING\n"
    else: toReturn += "MODE: SCOUT\n"
    return toReturn

def pos_decode(xpos, ypos):
    toReturn = "FACE: "
    if (xpos & 0B00000000 == 0B00000000):
        toReturn += "N\t"
    elif (xpos & 0B01000000 == 0B01000000):
        toReturn += "S\t"
    elif (xpos & 0B10000000 == 0B10000000):
        toReturn += "E\t"
    else: toReturn += "W\t"

    toReturn += "xpos: "
    xpos = xpos | 0B000000
    toReturn += int.from_bytes(xpos)
    toReturn += "\t"

    toReturn += "ypos: "
    toReturn += int.from_bytes(ypos)

    return toReturn

    


# Returns whether or not the command has been executed by the peripheral
# The left-most bit 
def is_command_executed(val):
    if (val & 0B10000000 == 0B10000000):
        return True

def encode_command(com):
    return bin(com)


async def run():

    print('SCOUT MASTER Central Service')
    print('Looking for SCOUT1 Device...')

    found = False
    devices = await discover()
    for d in devices:       
        if 'SCOUT1'in d.name:
            print('Found SCOUT1 Peripheral')
            found = True
            async with BleakClient(d.address) as client:
                print(f'Connected to {d.address}')
                while True:

                    # issue initial mode to peripheral ONCE
                    if (not scout1_init):
                        scout1_curr_command = encode_command(Command.SET_MODE_SCOUT)
                        await client.write_gatt_char(COMMAND_CHAR_UUID, scout1_curr_command, False)
                        print("set mode scout")
                        scout1_is_command_issued = True
                        scout1_init = True

                    # always decode status
                    val = int.from_bytes(await client.read_gatt_char(STATUS_CHAR_UUID), "little")
                    if (DEBUG_PRINT_STATUS):
                        print(status_decode(val))

                    # always decode position
                    xpos = await client.read_gatt_char(XPOS_CHAR_UUID)
                    ypos = await client.read_gatt_char(YPOS_CHAR_UUID)
                    print(pos_decode(xpos, ypos))

                    # check if scout1 performed command
                    if (scout1_is_command_issued):
                        val = await client.read_gatt_char(COMMAND_CHAR_UUID, "little")
                        if (is_command_executed(val)):
                            scout1_is_command_issued = False

                    # 
                    
                    # send instruction on condition
                    

    if not found:
        print('Could not find SCOUT1')


def my_handler():
    print('Stopping')
    for task in asyncio.Task.all_tasks():
        task.cancel()


loop = asyncio.get_event_loop()
loop.add_signal_handler(signal.SIGINT, my_handler)
try:
    loop.run_until_complete(run())
    #loop.run_forever()
    #tasks = Task.all_tasks()
except asyncio.CancelledError:
    print('Tasks have been cancelled')
#except KeyboardInterrupt:
    #print('\nReceived Keyboard Interrupt')
finally:
    loop.close()
    print('Program finished')