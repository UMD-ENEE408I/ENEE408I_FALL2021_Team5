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
from ssl import ALERT_DESCRIPTION_CERTIFICATE_UNOBTAINABLE

from bleak import BleakClient
from bleak import BleakScanner
from bleak import discover

from enum import Enum

# These values have been randomly generated - they must match between the Central and Peripheral devices
# Any changes you make here must be suitably made in the Arduino program as well

STATUS_CHAR_UUID = '19d10001-e8f2-537e-4f6c-d104768a1214'
COMMAND_CHAR_UUID = '19d10002-e8f2-537e-4f6c-d104768a1214'
XPOS_CHAR_UUID = '19d10003-e8f2-537e-4f6c-d104768a1214'
YPOS_CHAR_UUID = '19d10004-e8f2-537e-4f6c-d104768a1214'
DIRECTION_CHAR_UUID = '19d10005-e8f2-537e-4f6c-d104768a1214'
MAPPING_CHAR_UUID = '19d10006-e8f2-537e-4f6c-d104768a1214'

scout1_init = False
DEBUG_PRINT_STATUS = True # print out the status every time
DEBUG_POS_DECODE_DIRECTION = False
DEBUG_MAP_BOOLEANS = True # print out map booleans
DEBUG_MAPPING_DECODE = True # print out the mapping byte received
DECODE_MAPPING_CHARACTERISTIC = False
scout1_curr_command = 0B00000000

ROW = 50
COL = 50

MazeX = 25 
MazeY = 25 
prevMazeX = 25
prevMazeY = 25

Move_Count = 0

# N = 0, S = 1, E = 16, W = 17
Direction = 0

atIntersection = False
atRight = False
atLeft = False
atExit = False
travelledUnitLength = False
isForward = False
atDeadEnd = False

#Previous 
prevatIntersection = False
prevatRight = False
prevatLeft = False
prevatExit = False
prevtravelledUnitLength = False
previsForward = False
prevatDeadEnd = False

#__________________________________JACK'S MAZE MAPPING___________________

MazeMap =[["X" for i in range (ROW)] for i in range (COL)]
MazeMap[MazeX][MazeY] = "S"

def mapTheMaze():

    global MazeX
    global MazeY

    global atIntersection
    global atRight
    global atLeft
    global isForward

    #MazeMap =[["X" for i in range (ROW)] for i in range (COL)]
    #MazeMap[MazeX][MazeY] = "S"

    #Adjust X and Y Coordinates to the new posiotion
    #if (Direction == 0):
        #MazeY = MazeY - 1
    #elif (Direction == 1): 
        #MazeY = MazeY + 1; 
    #elif (Direction == 16):
        #MazeX = MazeX + 1;
    #elif (Direction == 17):
        #MazeX = MazeX - 1; 
        
    MazeMap[MazeY][MazeX] = "T"

    #Intersection
    if(atIntersection == True): 
        if (Direction == 0):
            MazeMap[MazeY][(MazeX)+1] = "T"; #Set IsNode field to the Left to True (East and West have Nodes)
            MazeMap[MazeY][(MazeX)-1] = "T"; #Set IsNode field to the Right to True
        elif (Direction == 1): 
            MazeMap[MazeY][(MazeX)+1] = "T"; #Set IsNode field to the Left to True (East and West have Nodes)
            MazeMap[MazeY][(MazeX)-1] = "T"; #Set IsNode field to the Right to True
        elif (Direction == 16):
            MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Left to True (North and South have nodes)
            MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Right to True
        elif (Direction == 17): 
            MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Left to True (North and South Have Nodes)
            MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Right to True

    #Right Turn
    elif ((atIntersection == False) and atRight == True):
        if (Direction == 0):
            MazeMap[MazeY][(MazeX)+1] = "T"; #Set IsNode field to the Right to True
        elif (Direction == 1): 
            MazeMap[MazeY][(MazeX)-1] = "T"; #Set IsNode field to the Right to True
        elif (Direction == 16):
            MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Left to True (North and South have nodes)
        elif (Direction == 17):
            MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Right to True 

    #Left Turn     
    elif ((not atIntersection) and atLeft == True):
        if (Direction == 0):
            MazeMap[MazeY][(MazeX)-1] = "T"; #Set IsNode field to the Right to True
        elif (Direction == 1): 
            MazeMap[MazeY][(MazeX)+1] = "T"; #Set IsNode field to the Right to True
        elif (Direction == 16):
            MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Left to True (North and South have nodes)
        elif (Direction == 17):
            MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Right to True  

    #Forward
    elif (travelledUnitLength == True):
        if (Direction == 0):
            MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Right to True
        elif (Direction == 1): 
            MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Right to True
        elif (Direction == 16):
            MazeMap[(MazeY)][(MazeX)+1] = "T"; #Set IsNode field to the Left to True (North and South have nodes)
        elif (Direction == 17):
            MazeMap[(MazeY)][(MazeX)-1] = "T"; #Set IsNode field to the Right to True  

    #Exit
    elif (atExit == True):
        MazeMap[MazeY][MazeX] = "E"; 
        
    if (DEBUG_MAP_BOOLEANS):
        toPrint = ""
        toPrint += "atIntersection: "
        toPrint += str(atIntersection)
        toPrint += "\t"
        toPrint += "atRight: "
        toPrint += str(atRight)
        toPrint += "\t"
        toPrint += "atLeft: "
        toPrint += str(atLeft)
        toPrint += "\t"
        toPrint += "isForward: "
        toPrint += str(isForward)
        print(toPrint)

    for r in MazeMap:
        for c in r:
            print(c,end = " ")
        print()  

#_______







on_value = bytearray([0x01])
off_value = bytearray([0x00])

class Command(Enum):
    MOUSE_STOP = 0
    MOVE_FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    TURN_AROUND = 4
    SET_MODE_SCOUT = 5
    SET_MODE_FOLLOWING = 6



def status_decode(val):
    global atIntersection
    global atRight
    global atLeft
    global atExit
    global travelledUnitLength
    global isForward
    global atDeadEnd

    #toReturn = ""
    #check mouse mode
    if (val & 0B10000000 == 0B10000000):
        #toReturn += "atExit\t"
        atExit = True
    else: 
        #toReturn += "!atExit\t"
        atExit = False
    if (val & 0B01000000 == 0B01000000):
        #toReturn += "atDeadEnd\t"
        atDeadEnd = True
    else: 
        #toReturn += "!atDeadEnd\t"
        toReturn = False
    if (val & 0B00100000 == 0B00100000):
        #toReturn += "isForward\t"
        isForward = True
    else: 
        #toReturn += "!isForward\t"
        isForward = False
    if (val & 0B00010000 == 0B00010000):
        #toReturn += "atLeft\t"
        atLeft = True
    else: 
        #toReturn += "!atLeft\t"
        atLeft = False
    if (val & 0B00001000 == 0B00001000):
        #toReturn += "atRight\t"
        atRight = True
    else: 
        #toReturn += "!atRight\t"
        atRight = False
    if (val & 0B00000100 == 0B00000100):
        #toReturn += "atIntersection\t"
        atIntersection = True
    else: 
        #toReturn += "!atIntersection\t"
        atIntersection = False
    if (val & 0B00000010 == 0B00000010):
        #toReturn += "onWhiteLine\t"
        travelledUnitLength = True
    else:
        travelledUnitLength = False
    #else: toReturn += "!onWhiteLine\t"
    #if (val & 0B00000001 == 0B00000001):
        #toReturn += "MODE: FOLLOWING\n"
    #else: toReturn += "MODE: SCOUT\n"
    #return toReturn

def mapping_decode(val):
    global atIntersection
    global atRight
    global atLeft
    global atExit
    global travelledUnitLength
    global isForward
    global atDeadEnd

    #if (DEBUG_MAPPING_DECODE):
        #print(val)

    if ((val & 0B00000001) == 0B00000001):
        atIntersection = True
    else: atIntersection = False
    if ((val & 0B00000010) == 0x00000010):
        atRight = True
    else: atRight = False
    if ((val & 0B00000100) == 0x00000100):
        atLeft = True
    else: atLeft = False
    if ((val & 0B00001000) == 0x00001000):
        atExit = True
    else: atExit = False
    if ((val & 0B00010000) == 0x00010000):
        travelledUnitLength = True
    else: travelledUnitLength = False
    if ((val & 0B00100000) == 0x00100000):
        isForward = True
    else: isForward = False
    if ((val & 0B01000000) == 0x01000000):
        atDeadEnd = True
    else: atDeadEnd = False

def pos_print(xpos, ypos, dir):

    
    toReturn = "FACE: "
    toReturn += str(dir)
    toReturn += "\t"

    if (DEBUG_POS_DECODE_DIRECTION):
        toReturn += str(dir)
        toReturn += "\t"

        #toReturn += str(direction)

    """ if (direction == bytearray([int(0x00, 16)])):
        toReturn += "N\t"
    elif (direction == bytearray([int(0x01, 16)])):
        toReturn += "S\t"
    elif (direction == bytearray([int(0x10, 16)])):
        toReturn += "E\t"
    else: 
        toReturn += "W\t" """


    toReturn += "xpos: "
    #xpos = xpos | 0B000000
    toReturn += str(xpos) #int.from_bytes(xpos)
    toReturn += "\t"

    toReturn += "ypos: "
    toReturn += str(ypos) #int.from_bytes(ypos)

    print(toReturn)

def pos_decode(xpos, ypos, dir):

    
    #toReturn = "FACE: "

    if (DEBUG_POS_DECODE_DIRECTION):
        toReturn += str(dir)
        toReturn += "\t"

        #toReturn += str(direction)

    """ if (direction == bytearray([int(0x00, 16)])):
        toReturn += "N\t"
    elif (direction == bytearray([int(0x01, 16)])):
        toReturn += "S\t"
    elif (direction == bytearray([int(0x10, 16)])):
        toReturn += "E\t"
    else: 
        toReturn += "W\t" """


    #toReturn += "xpos: "
    #xpos = xpos | 0B000000
    #toReturn += str(xpos) #int.from_bytes(xpos)
    #toReturn += "\t"

    #toReturn += "ypos: "
    #toReturn += str(ypos) #int.from_bytes(ypos)

    global MazeX
    global MazeY
    
    global prevMazeX
    prevMazeX = MazeX
    
    MazeX = xpos

    global prevMazeY
    prevMazeY = MazeY

    MazeY = ypos

    #global Direction
    #Direction = dir

    #print(toReturn)

    
"""  if (int.from_bytes(bytes(direction), "little") == 0):
        toReturn += "N\t"
    elif (int.from_bytes(bytes(direction), "little") == 1):
        toReturn += "S\t"
    elif (int.from_bytes(bytes(direction), "little") == 16):
        toReturn += "E\t"
    else: 
        toReturn += "W\t" """

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
                    """ if (not scout1_init):
                        scout1_curr_command = encode_command(Command.SET_MODE_SCOUT)
                        await client.write_gatt_char(COMMAND_CHAR_UUID, scout1_curr_command, False)
                        print("set mode scout")
                        scout1_is_command_issued = True
                        scout1_init = True """
                        
                    #Set Up previous Turn Infor
                    global atIntersection
                    global atRight
                    global atLeft
                    global atExit
                    global isForward
                    global atDeadEnd
                    global travelledUnitLength
                    
                    global prevatIntersection
                    prevatIntersection = atIntersection
                    global prevatRight
                    prevatRight = atRight
                    global prevatLeft
                    prevatLeft = atLeft
                    global prevatExit
                    prevatExit = atExit
                    global previsForward
                    previsForward = isForward
                    global prevatDeadEnd
                    prevatDeadEnd = atDeadEnd
                    global prevtravelledUnitLength
                    prevtravelledUnitLength = travelledUnitLength

                    # always decode status
                    status_val = int.from_bytes(await client.read_gatt_char(STATUS_CHAR_UUID), "little")
                    if (DEBUG_PRINT_STATUS):
                        #print(status_decode(status_val))
                        status_decode(status_val)

                    # always decode position
                    xpos = int.from_bytes(await client.read_gatt_char(XPOS_CHAR_UUID), "little")
                    ypos = int.from_bytes(await client.read_gatt_char(YPOS_CHAR_UUID), "little")
                    dir = int.from_bytes(await client.read_gatt_char(DIRECTION_CHAR_UUID), "little")
                    global MazeX
                    global MazeY
                    global prevMazeX
                    global prevMazeY
                    global Direction
                    global Move_Count
                    
                    if(Move_Count == 0):
                        prevMazeX = MazeX
                        prevMazeY = MazeY


                    MazeX = xpos
                    MazeY = ypos
                    Direction = dir
                    #pos_decode(xpos, ypos, direction)

                    # always decode mapping characteristic
                    if (DECODE_MAPPING_CHARACTERISTIC):
                        mapping_val = (await client.read_gatt_char(MAPPING_CHAR_UUID), "little")
                        mapping_decode(mapping_val)

                    # check if scout1 performed command
                    
                    """ if (scout1_is_command_issued):
                        val = await client.read_gatt_char(COMMAND_CHAR_UUID, "little")
                        if (is_command_executed(val)):
                            scout1_is_command_issued = False """
 
                    # 
                    
                    # send instruction on condition

                    #
                    
                     
                    
                    #if (not(MazeX == prevMazeX) or not(MazeY == prevMazeY)):
                        #Move_Count = 1
                    if((atIntersection == False) and (atDeadEnd == False) and (atRight == False) and (atLeft == False) and (atExit == False)): 
                        Move_Count = 0
                        pos_print(MazeX, MazeY, Direction)
                        mapTheMaze()
                    elif(not(atIntersection == prevatIntersection) and not(atDeadEnd == prevatDeadEnd) and not(atRight == prevatRight) and not(atLeft == prevatLeft) and not(atExit == prevatExit)):
                        Move_Count = 0
                        pos_print(MazeX, MazeY, Direction)
                        mapTheMaze()
                    

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