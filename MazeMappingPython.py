
ROW = 50
COL = 50

MazeX = 25; 
MazeY = 25; 
Direction = "N"

atIntersection = True
atRight = True
atLeft = True
travelledUnitLength = False
atExit = False


MazeMap =[["X" for i in range (ROW)] for i in range (COL)]
MazeMap[MazeX][MazeY] = "S"

#Adjust X and Y Coordinates to the new posiotion
if (Direction == "N"):
    MazeY = MazeY - 1
elif (Direction == "S"): 
    MazeY = MazeY + 1; 
elif (Direction == "E"):
    MazeX = MazeX + 1;
elif (Direction == "W"):
    MazeX = MazeX - 1; 
      
MazeMap[MazeY][MazeX] = "T"
    
#Intersection
if(atIntersection == True): 
    if (Direction == "N"):
        MazeMap[MazeY][(MazeX)+1] = "T"; #Set IsNode field to the Left to True (East and West have Nodes)
        MazeMap[MazeY][(MazeX)-1] = "T"; #Set IsNode field to the Right to True
    elif (Direction == "S"): 
        MazeMap[MazeY][(MazeX)+1] = "T"; #Set IsNode field to the Left to True (East and West have Nodes)
        MazeMap[MazeY][(MazeX)-1] = "T"; #Set IsNode field to the Right to True
    elif (Direction == "E"):
        MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Left to True (North and South have nodes)
        MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Right to True
    elif (Direction == "W"): 
        MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Left to True (North and South Have Nodes)
        MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Right to True
    
#Right Turn
elif ((atIntersection == False) and atRight == True):
    if (Direction == "N"):
        MazeMap[MazeY][(MazeX)+1] = "T"; #Set IsNode field to the Right to True
    elif (Direction == "S"): 
        MazeMap[MazeY][(MazeX)-1] = "T"; #Set IsNode field to the Right to True
    elif (Direction == "E"):
        MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Left to True (North and South have nodes)
    elif (Direction == "W"):
        MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Right to True 
       
#Left Turn     
elif ((not atIntersection) and atLeft == True):
    if (Direction == "N"):
        MazeMap[MazeY][(MazeX)-1] = "T"; #Set IsNode field to the Right to True
    elif (Direction == "S"): 
        MazeMap[MazeY][(MazeX)+1] = "T"; #Set IsNode field to the Right to True
    elif (Direction == "E"):
        MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Left to True (North and South have nodes)
    elif (Direction == "W"):
        MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Right to True        
   
#Forward
elif (travelledUnitLength == True):
    if (Direction == "N"):
        MazeMap[(MazeY)-1][MazeX] = "T"; #Set IsNode field to the Right to True
    elif (Direction == "S"): 
        MazeMap[(MazeY)+1][MazeX] = "T"; #Set IsNode field to the Right to True
    elif (Direction == "E"):
        MazeMap[(MazeY)][(MazeX)+1] = "T"; #Set IsNode field to the Left to True (North and South have nodes)
    elif (Direction == "W"):
        MazeMap[(MazeY)][(MazeX)-1] = "T"; #Set IsNode field to the Right to True        
   
#Exit
elif (atExit == True):
    MazeMap[MazeY][MazeX] = "E"; 
    
for r in MazeMap:
   for c in r:
      print(c,end = " ")
   print()

   