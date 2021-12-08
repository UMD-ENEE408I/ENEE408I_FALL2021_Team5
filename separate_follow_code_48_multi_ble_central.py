# -*- coding: utf-8 -*-
"""
/******************************************************************* 
  Scout Service Central
  ENEE408I Team 5
    Last edited: 11/30/21 5:02PM
  This code receives a byte from the mouse, decodes it, and prints it
 */
"""
from collections import deque as queue 

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
scouting_finished = False

ROW = 50
COL = 50

MazeX_Start = 5
MazeY_Start = 49
MazeX = MazeX_Start
MazeY = MazeY_Start
prevMazeX = MazeX_Start
prevMazeY = MazeY_Start
prevMazeX_2 = 0
prevMazeY_2 = 0
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

#Maze Solving Variables
#Direction vectors
dRow =[-1, 0, 1, 0] 
dCol =[0, 1, 0, -1] 
#Declare the previous array
previous =[[-1 for i in range (ROW)] for i in range (COL)]
path_Exit2Start = [0 for i in range (100)]
path_Start2Exit = [0 for i in range (100)]

Command_Array = [0 for i in range(100)]
Command_Counter = 0

def is_command_empty(theByte):
    if (theByte == 0B00000000):
        return True
    return False

def encode_command(mouseNum, command):
    toSend = 0B00000000
    if (mouseNum == 1):
        toSend = 0B01000000
    elif (mouseNum == 2):
        toSend = 0B10000000
    else:
        toSend = 0B11000000
    
    toSend = toSend | (command)

    return toSend

#-----------------------------JACK MAZE SOLVING--------------------------
#Find Start Node and End Node
def search(mat, n, x):
    if(n == 0):
        return -1
     
    # Traverse through the matrix  
    for i in range(n):
        for j in range(n):
             
            # If the element is found
            if(mat[i][j] == x):
                print("Element found at (", i, ",", j, ")")
                return (i*ROW) + (j)
     
    print(" Element not found")
    return 0
    
#Function to Find the Shortest Path to the exit
def path2exit(previous, s, e):
    steps = 0
    X = e//ROW  
    Y = e%ROW
    k = 0;
    i = 0; 
    at = e 
    
    while not (at == -1): 
        path_Exit2Start[i] = at
        at = previous[X][Y]
        X = at//ROW 
        Y = at%ROW 
        i = i+1
    
    
    steps = i; 
    
    X = e//ROW;   #Preforms Floor Devision
    Y = e%ROW;
    at = e; 
    path_Start2Exit[steps-1] = at; 
    
    #for(i = (steps-2); i>=0; i--){
    for i in range(steps-1):
        path_Start2Exit[(steps-2)-i] = previous[X][Y]; 
        at = previous[X][Y]; 
        X = at//ROW;
        Y = at%ROW; 
    
    
    return steps; 
    
# Function to check if a cell
# is be visited or not
def isValid(grid, vis, row, col):
   
    # If cell lies out of bounds
    if (row < 0 or col < 0 or row >= ROW or col >= COL):
        return False
 
    # If cell is already visited
    if (vis[row][col]):
        return False
     
    #If there is a Wall   
    if(grid[row][col] == '-'): 
        return False 
    
    # Otherwise
    return True
    
# Function to perform the BFS traversal
def BFS(grid, vis, row, col):
   
    # Stores indices of the matrix cells
    q = queue()
 
    # Mark the starting cell as visited
    # and push it into the queue
    q.append(( row, col ))
    vis[row][col] = True
 
    # Iterate while the queue
    # is not empty
    while (len(q) > 0):
        cell = q.popleft()
        x = cell[0]
        y = cell[1]
        print(grid[x][y], end = " ")
 
        #q.pop()
 
        # Go to the adjacent cells
        for i in range(4):
            adjx = x + dRow[i]
            adjy = y + dCol[i]
            if (isValid(grid, vis, adjx, adjy)):
                q.append((adjx, adjy))
                vis[adjx][adjy] = True
                previous[adjx][adjy] = (x*ROW) + (y);

#__________________________________JACK'S MAZE MAPPING___________________

MazeMap =[["-" for i in range (ROW)] for i in range (COL)]
MazeMap[MazeY][MazeX] = "S"

def mapPosition():
    global MazeX
    global MazeY

    MazeMap[MazeY][MazeX] = "T"
    
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
    
    #Exit
    if (atExit == True):
        MazeMap[MazeY][MazeX] = "E"; 

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

    #Straight
    #elif (travelledUnitLength == True):
        #if (Direction == 0):
            #MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Right to True
        #elif (Direction == 1): 
            #MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Right to True
        #elif (Direction == 16):
            #MazeMap[(MazeY)][(MazeX)+1] = "T"; #Set IsNode field to the Left to True (North and South have nodes)
        #elif (Direction == 17):
            #MazeMap[(MazeY)][(MazeX)-1] = "T"; #Set IsNode field to the Right to True  

        
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
        toPrint += "\t"
        toPrint += "Travelled Full Unit: "
        toPrint += str(travelledUnitLength)
        toPrint += "\t"
        toPrint += "AtDeadEnd: "
        toPrint += str(atDeadEnd)
        print(toPrint)

    for r in MazeMap:
        for c in r:
            print(c,end = " ")
        print()  

#_____________________________________________________________



on_value = bytearray([0x01])
off_value = bytearray([0x00])

class Command(Enum):
    MOUSE_STOP = 0
    MOVE_FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    TURN_AROUND = 4
    READY = 5
    #SET_MODE_FOLLOWING = 6



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

        print('FOLLOW MASTER Central Service')
        print('Looking for FOLLOWER Device...')

        found = False
        devices = await discover()

        for d in devices:       
            if 'FOLLOWER'in d.name:
                print('Found FOLLOWER Peripheral')
                found = True
                async with BleakClient(d.address, use_cached=False) as client:
                    print(f'Connected to {d.address}')
                    finished  = False
                    while (not finished):
                        print("inside while true")
                        # check instruction
                        val = int.from_bytes(await client.read_gatt_char(COMMAND_CHAR_UUID), "little")
                        if (val == 0):
                            print("command is empty")
                            for r in range(Command_Counter):
                                #print("command counter:")
                                #print(Command_Counter)                               
                                print("encoding a command")
                                print(Command_Array[r])
                                # encode command to byte
                                #toSend = encode_command(r)
                                # send command to mouse
                                await client.write_gatt_char(COMMAND_CHAR_UUID, Command_Array[r], False)
                                # wait for mouse to receive and execute command
                                command_received_by_mouse = False
                                while (not command_received_by_mouse):
                                    print("waiting for mouse to receive the sent command")
                                    val = int.from_bytes(await client.read_gatt_char(COMMAND_CHAR_UUID), "little")
                                    if (val == 10):
                                        print("mouse received command")
                                        command_executed_by_mouse = False
                                        while(not command_executed_by_mouse):
                                            val = int.from_bytes(await client.read_gatt_char(COMMAND_CHAR_UUID), "little")
                                            if (val == 20):
                                                print("command executed")
                                                command_executed_by_mouse = True
                                                command_received_by_mouse = True
                                        
                                #print("REPLACE THIS LINE WITH YOUR OWN IMPLEMENTATION")
                                finished = True
                        
        if not found:
            print('Could not find FOLLOWER')
        



    

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