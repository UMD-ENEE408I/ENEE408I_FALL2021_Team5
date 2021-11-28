import asyncio
from bleak import BleakClient
import struct
import time

from bleak import BleakScanner
#from bleak import discover

async def run():
    global RED, GREEN, BLUE
    print('ProtoStax Arduino Nano BLE LED Peripheral Central Service')
    print('Looking for Arduino Nano 33 BLE Sense Peripheral Device...')
    found = False
    devices = await discover()
    for d in devices: 
        if 'Arduino Nano 33 BLE Sense'in d.name:
            print('Found Arduino Nano 33 BLE Sense Peripheral')
        found = True
        async with BleakClient(d.address) as client:
            print(f'Connected to {d.address}')
            val = await client.read_gatt_char(RED_LED_UUID)
            if (val == on_value):
                print ('RED ON')
                RED = True
            else:
                print ('RED OFF')
                RED = False
            # ... do similar stuff for GREEN and BLUE
            while True:
                await setColor(client)
        if not found:
            print('Could not find Arduino Nano 33 BLE Sense Peripheral')

async def setColor(client):
    global RED
    global GREEN
    global BLUE
    
    val = input('Enter rgb to toggle red, green and blue LEDs :')
    print(val)
    if ('r' in val):
        RED = not RED
        await client.write_gatt_char(RED_LED_UUID, getValue(RED))
    #... do similar stuff for GREEN and BLUE
    if ('g' in val):
        GREEN = not GREEN
        await client.write_gatt_char(GREEN_LED_UUID, getValue(GREEN))
    if ('b' in val):
        BLUE = not BLUE
        await client.write_gatt_char(BLUE_LED_UUID, getValue(BLUE))