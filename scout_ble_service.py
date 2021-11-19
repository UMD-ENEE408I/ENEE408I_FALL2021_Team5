import asyncio
from bleak import BleakClient
import struct
import time

#to import from git to jetson use:
# $ wget github/raw/url/link/thing

#address = "50:C7:CA:6F:7A:4F"
address = "3A:A8:FE:D9:13:BE"
MOTOR_CHAR_UUID = "0fe79935-cd39-480a-8a44-06b70f36f249"

SCOUT_SERVICE_UUID = "19d10000-e8f2-537e-4f6c-d104768a1214"
COUNTER_CHAR_UUID = "19d10001-e8f2-537e-4f6c-d104768a1214"

async def run(address):
    async with BleakClient(address) as client:
        counter_val = 0

        t_start = time.time()
        await client.write_gatt_char(COUNTER_CHAR_UUID, struct.pack('hh', counter_val))

        value = await client.read_gatt_char(COUNTER_CHAR_UUID)
        counter_val = struct.unpack('hh', value)
        print("counter: {}".format(counter_val))
        t_end = time.time()

        """ left_value = -15
        right_value = -20

        t_start = time.time()
        for i in range(40):
            await client.write_gatt_char(MOTOR_CHAR_UUID, struct.pack('hh', left_value, right_value))

            value = await client.read_gatt_char(MOTOR_CHAR_UUID)
            (left_value, right_value) = struct.unpack('hh', value)
            print("value: {} {}".format(left_value, right_value))

            left_value += 1
            right_value += 1
        t_end = time.time()

        print('{} Hz'.format(40*2 / (t_end-t_start))) """

loop = asyncio.get_event_loop()
loop.run_until_complete(run(address))