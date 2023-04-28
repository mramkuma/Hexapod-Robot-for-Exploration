from matplotlib import markers, offsetbox, quiver
from matplotlib.cbook import print_cycles
from numpy import Infinity, append, empty, extract, size, source, var
import numpy as np
from pyparsing import col




class Node:
    def __init__(self, row, col, is_obs, h="Manhattan"):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node
        self.visited = False



def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    
    tot_row = len(grid)
    tot_col = len(grid[0])
    
    # Initializing visited, parent, distance, heuristic and f array
    visited = [[False for i in range(tot_col)]for j in range(tot_row)]
    parent = [[[0,0] for i in range(tot_col)]for j in range(tot_row)]
    distance = [[Infinity for i in range(tot_col)]for j in range(tot_row)]
    heuristic = [[0 for i in range(tot_col)]for j in range(tot_row)]
    f = [[Infinity for i in range(tot_col)]for j in range(tot_row)]
    
    for i in range (tot_row):
        for j in range(tot_col):
            heuristic[i][j] = abs(i-goal[0]) + abs(j-goal[1])


    # Setting parent, visited, Distance, Heuristic nodes for the start node
    parent[start[0]][start[1]] = [-1,-1]
    visited[start[0]][start[1]] = True
    distance[start[0]][start[1]] = 0 
    f[start[0]][start[1]] = heuristic[start[0]][start[1]]

    open = []
    
  
    open.append(start) # Appending goal to Queue
    
    while open:
        row,col = calculate_min(open,f)
        
        open.remove([row,col]) # Removing element with lowest f function
        
        steps+=1
        
        # Getting unvisited neighbors    
        neighbor,visited = getneighbors(tot_row,tot_col,row,col,visited,grid)
        
        for x,y in neighbor:
            source_distance = distance[row][col]
            node_distance = abs(x-row) + abs(y-col)
            total_distance = source_distance + node_distance
            
                
            if total_distance<distance[x][y]:
                distance[x][y] = total_distance
                parent[x][y] = [row,col]
                f[x][y] = total_distance + heuristic[x][y]
                
                if [x,y] not in open:
                    open.append([x,y])
                    
                # Finding Goal in neigbor nodes    
                if [x,y]==goal:
                    steps+=1
                    open = []
                    found = True 
    
    # Getting Path        
    path = extractpath(parent,start,goal) 

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        path = []
        steps = 0
        print("No path found")
    return path, steps


## Functions Used for BFS, DFS, Dijkstra and Astar

# Function to calculate the minimum of distance and f function used in 
# dijkstra and astar algorithm. Returns row and column of smallest 
# element of array

def calculate_min(open, f): 
    temp = Infinity
    for x,y in open:
        if f[x][y]<temp:
            temp = f[x][y]
            row = x
            col = y
            
    return row,col

# Function to check obstacle
def isobstacle(grid,row,col):
    if grid[row][col] ==1:
        return True

# Function to get unvisited neigbors of given node. Inputs are row and col
# of input node along with total size of the grid. Visited array is also 
# updated once neighbors are returned
  
def getneighbors(tot_row,tot_col,row,col,visited,grid):
    
    neighbor = []
    
    neighbor.append([row,col+1])
    neighbor.append([row+1,col])
    neighbor.append([row,col-1])
    neighbor.append([row-1,col])
    
    
    for num,xy in enumerate(neighbor):
        x = xy[0]
        y = xy[1]
        if x<0 or x>tot_row-1:
            neighbor[num] = [-1,-1]
        elif y<0 or y>tot_col-1:
            neighbor[num] = [-1,-1]
        elif grid[x][y] ==1:
            neighbor[num] = [-1,-1]
        elif visited[x][y] == True:
            neighbor[num] = [-1,-1]
        else:
            visited[x][y] = True
            
    try:
            while True:
                neighbor.remove([-1,-1])
    except ValueError:
            pass
            
         
    return neighbor,visited

# Function to extract path from parent array, start and goal
def extractpath(parent,start,goal):
    temp = parent[goal[0]][goal[1]]
    path = []
    current = temp
    while True:
        path.append(current)
        current = parent[current[0]][current[1]]
        
        if current[0]==start[0] and current[1]==start[1]:
            break
        
    path.append(start)
    path.reverse()
    path.append(goal)
       
    return path

    


