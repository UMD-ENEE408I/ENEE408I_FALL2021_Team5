# -*- coding: utf-8 -*-
"""
/******************************************************************* 
  ProtoStax Arduino Nano 33 BLE Sense RGB LED Control Central Device
  This is a example sketch for controlling the RGB LED on an 
  Arduino Nano 33 BLE Sense with Bluetooth over Python  
   
  Items used:
  Arduino Nano 33 BLE Sense
  ProtoStax for BreadBoard/Custom Boards - 
      - to house and protect the Nano and allow for other circuitry 
      --> https://www.protostax.com/collections/all/products/protostax-for-breadboard
  
  The Nano publishes a Bluetooth LE Client profile with Characteristics for the Red, Green, 
  and Blue components of the onboard RGB LED. These can be read and written to
  control the LED colors.
  This program toggles the R,G,B LEDs based on user input. Run the python program from your computer
  (PC, Mac or Linux) that has Bluetooth support and the requisite python packages - 
  you can then read and set the on/off states of the 3 colors. 
  
  The Red, Green and Blue colors of the onboard RGB LED can only be turned on or off. 
  It is not possible to use PWM to mix colors, unfortunately, based on how the Arduino 
  Nano BLE Sense board is configured.
  
  We write a value of 1 to turn on a color and 0 to turn it off. The user inputs 
  a string that can contain r,g,b (or any combination) and those colors will be toggled. 
  The Arduino Nano 33 BLE Sense is chockful of other sensors - you can similarly expose 
  those sensors data as Characteristics
 
  Written by Sridhar Rajagopal for ProtoStax
  BSD license. All text above must be included in any redistribution
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

RED = False
GREEN = False
BLUE = False


def getValue(on):
    if on:
        return on_value
    else:
        return off_value

async def setColor(client):
    global RED, GREEN, BLUE
    val = input('Enter rgb to toggle red, green and blue LEDs :')
    print(val)

    if ('r' in val):
        RED = not RED
        await client.write_gatt_char(RED_LED_UUID, getValue(RED))
    if ('g'in val):
        GREEN = not GREEN
        await client.write_gatt_char(GREEN_LED_UUID, getValue(GREEN))
    if ('b' in val):
        BLUE = not BLUE
        await client.write_gatt_char(BLUE_LED_UUID, getValue(BLUE))
    

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
                    print(int.from_bytes(val))
                    

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