#Python3 program for the above approach
from collections import deque as queue 
 
#Row and Column Function
ROW = 11
COL = 11

 
#Direction vectors
dRow =[-1, 0, 1, 0] 
dCol =[0, 1, 0, -1] 
#Declare the previous array
previous =[[-1 for i in range (ROW)] for i in range (COL)]
path_Exit2Start = [0 for i in range (100)]
path_Start2Exit = [0 for i in range (100)]



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
    if(grid[row][col] == 'X'): 
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
 
#Driver Code
if __name__ == '__main__':
 
	      # Given input matrix
	      grid = [['X', 'T', 'T', 'X', 'X', 'X', 'X', 'X','X', 'X', 'X'],
                           [ 'X', 'T', 'T', 'X', 'X', 'X', 'X', 'X','X', 'X', 'E'],
                           [ 'X', 'T', 'T', 'X', 'X', 'X', 'T', 'T','T', 'T', 'T'],
                           [ 'X', 'T', 'T', 'X', 'X', 'X', 'T', 'X','X', 'T', 'X'],
                           [ 'X', 'T', 'T', 'X', 'X', 'X', 'T', 'T','T', 'T', 'X'],
                           [ 'X', 'T', 'T', 'T', 'T', 'S', 'T', 'X','T', 'X', 'X'],
                           [ 'T', 'T', 'T', 'X', 'T', 'X', 'X', 'X','T', 'X', 'X'],
                           [ 'X', 'T', 'T', 'X', 'T', 'X', 'X', 'X','X', 'X', 'X'],
                           [ 'X', 'T', 'T', 'X', 'T', 'X', 'X', 'X','X', 'X', 'X'],
                           [ 'X', 'T', 'T', 'X', 'T', 'X', 'X', 'X','X', 'X', 'X'],
                           [ 'X', 'T', 'T', 'X', 'T', 'T', 'T', 'T','T', 'T', 'T']];
 
#Declare the visited array
vis =[[False for i in range (ROW)] for i in range (COL)]
  
#vis, False, sizeof vis)
    
BFS (grid, vis, 5, 5) 

#Print Grid Array
print("\nMaze Grid Array:\n")
for r in grid:
   for c in r:
      print(c,end = " ")
   print()
   
#Print Previos Node Array
print("\nPrevious Node from BFS Array:\n")
for r in previous:
   for c in r:
      print(c,end = " ")
   print()
   
        
#Find the Path from S to E. 
s = 60; #Center of the Array
e = 21; #End of the Array (will change for the map)
length = path2exit(previous, s, e); 
      
print("\nShow path from exit to start:\n");
for r in range(length):
     print(path_Exit2Start[r], " ")

print("\nShow path from start to exit:\n");
for r in range(length):
     print(path_Start2Exit[r], " ")
   
         
      