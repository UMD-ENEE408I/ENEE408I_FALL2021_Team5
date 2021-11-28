
 
#include <bits/stdc++.h>
using namespace std;
 
#define ROW 11
#define COL 11
 
// Direction vectors
int dRow[] = { -1, 0, 1, 0 };
int dCol[] = { 0, 1, 0, -1 };
int previous[ROW][COL] = {-1}; 
int path_Exit2Start[100]; 
int path_Start2Exit[100]; 


 //Function to Find the Shortest Path to the exit
int path2exit(int previous[][COL], int s, int e){ 
    int steps = 0; 
    int X = e/11;  
    int Y = e%11; 
    int k,i = 0; 
    
    for(int at = e; at!= -1; at = previous[X][Y]){ 
        path_Exit2Start[i] = at; 
        X = at/11; 
        Y = at%11; 
        i++; 
    } 
    
    steps = i; 
    
    X = e/11;  
    Y = e%11;
    int at = e;
    path_Start2Exit[steps-1] = at; 
    
    for(i = (steps-2); i>=0; i--){
        path_Start2Exit[i] = previous[X][Y]; 
        at = previous[X][Y]; 
        X = at/11;
        Y = at%11; 
    }
    
    return steps; 
}

// Function to check if a cell
// is be visited or not
bool isValid(char grid[][COL], bool vis[][COL],
             int row, int col)
{
    // If cell lies out of bounds
    if (row < 0 || col < 0
        || row >= ROW || col >= COL)
        return false;
 
    // If cell is already visited
    if (vis[row][col])
        return false;
    
    if(grid[row][col] == 'X'){
        return false; 
    }
 
    // Otherwise
    return true;
}
 
// Function to perform the BFS traversal
void BFS(char grid[][COL], bool vis[][COL],
         int row, int col)
{
    // Stores indices of the matrix cells
    queue<pair<int, int> > q;
 
    // Mark the starting cell as visited
    // and push it into the queue
    q.push({ row, col });
    vis[row][col] = true;
 
    // Iterate while the queue
    // is not empty
    while (!q.empty()) {
 
        pair<int, int> cell = q.front();
        int x = cell.first;
        int y = cell.second;
 
        cout << grid[x][y] << " ";
 
        q.pop();
        
      
 
        // Go to the adjacent cells
        for (int i = 0; i < 4; i++) {
 
            int adjx = x + dRow[i];
            int adjy = y + dCol[i];
 
            if (isValid(grid, vis, adjx, adjy)) {
                q.push({ adjx, adjy });
                vis[adjx][adjy] = true;
                previous[adjx][adjy] = (x*11) + (y);
            }
        }
    }
}
 
// Driver Code
int main()
{
    
int k, j; 
//Set all Values in Previous Array to Zero
for (k = 0; k < ROW; k++) {
    for (j = 0; j < COL; j++) {
        previous[k][j] = -1;
    }
}

    // Given input matrix
    char grid[ROW][COL] = { { 'X', 'T', 'T', 'X', 'X', 'X', 'X', 'X','X', 'X', 'X'},
                           { 'X', 'T', 'T', 'X', 'X', 'X', 'X', 'X','X', 'X', 'E'},
                           { 'X', 'T', 'T', 'X', 'X', 'X', 'T', 'T','T', 'T', 'T'},
                           { 'X', 'T', 'T', 'X', 'X', 'X', 'T', 'X','X', 'T', 'X'},
                           { 'X', 'T', 'T', 'X', 'X', 'X', 'T', 'T','T', 'T', 'X'},
                           { 'X', 'T', 'T', 'T', 'T', 'S', 'T', 'X','T', 'X', 'X'},
                           { 'T', 'T', 'T', 'X', 'T', 'X', 'X', 'X','T', 'X', 'X'},
                           { 'X', 'T', 'T', 'X', 'T', 'X', 'X', 'X','X', 'X', 'X'},
                           { 'X', 'T', 'T', 'X', 'T', 'X', 'X', 'X','X', 'X', 'X'},
                           { 'X', 'T', 'T', 'X', 'T', 'T', 'T', 'T','T', 'T', 'T'},};
 
    // Declare the visited array
    bool vis[ROW][COL];
    memset(vis, false, sizeof vis);
    
    BFS(grid, vis, 5, 5);
 
    printf("\n\nCreating 2D Array of Previos Nodes Using BFS\n");
    printf("\n");
    printf("Array sized [%i,%i] created.\n\n", COL, ROW);
    printf("Nodes are numbered By the Formula ((x*rows) + (y))\n\n");
    
      // print contents of the array2D
      printf("Array contents: \n");
    
    //Previous Node Matrix for Breath First Search
      for (int row = 0; row < ROW; row++)
      {
            for (int col = 0; col < COL; col++)
            {
                  printf("%i,", previous[row][col]);
            }
            printf("\n");
      }
      
      //Find the Path from S to E. 
      int s = 60; //Center of the Array
      int e = 21; //End of the Array
      int length = path2exit(previous, s, e); 
      
      
      printf("\nShow path from exit to start:\n");
      for(int l = 0; l < length; l++){
          printf("%d ", path_Exit2Start[l]); 
      }
      
      
      printf("\n\nShow path from start to exit:\n");
      for(int l = 0; l < length; l++){
          printf("%d ", path_Start2Exit[l]); 
      }
      
      
      
    return 0;
}