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

# These values have been randomly generated - they must match between the Central and Peripheral devices
# Any changes you make here must be suitably made in the Arduino program as well

RED_LED_UUID = '13012F01-F8C3-4F4A-A8F4-15CD926DA146'
GREEN_LED_UUID = '13012F02-F8C3-4F4A-A8F4-15CD926DA146'
BLUE_LED_UUID = '13012F03-F8C3-4F4A-A8F4-15CD926DA146'

SCOUT_SERVICE_CHAR_UUID = '19d10001-e8f2-537e-4f6c-d104768a1214'

on_value = bytearray([0x01])
off_value = bytearray([0x00])

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
                    val = int.from_bytes(await client.read_gatt_char(SCOUT_SERVICE_CHAR_UUID), "little")
                    print(status_decode(val))
                    

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