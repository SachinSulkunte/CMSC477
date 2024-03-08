import queue
from scipy.interpolate import make_interp_spline
import matplotlib.pyplot as plt
import numpy as np

def ucs(graph, start, goal):
    visited = set()
    frontier = queue.PriorityQueue()  # Using a priority queue
    frontier.put((0, start, [start]))  # Adding start node with cost 0

    while not frontier.empty():
        cost, node, path = frontier.get()
        if node not in visited:
            visited.add(node)

            if node == goal:
                return path  # Return the path to the goal

            # Explore neighbors
            for neighbor, neighbor_cost in graph[node].items():
                if neighbor not in visited:
                    new_cost = cost + neighbor_cost
                    frontier.put((new_cost, neighbor, path + [neighbor]))  # Add to the priority queue

    return None  # Goal not found

def breakxy(path):
    x_coord = []
    y_coord = []
    for pos in path:
        x_coord.append(pos.split(' ')[0])
        y_coord.append(pos.split(' ')[1])
    
    
    print(y_coord)
    print(x_coord)

    x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
    y = [6, 7, 8, 7, 6, 5, 6, 7, 8, 7, 6]
    xx = np.linspace(0, 11, 50)
    bspline = make_interp_spline(x, y, 3)
    plt.plot(xx, bspline(xx), '-', label='spline')
    plt.plot(x, y, 'o', label="original")
    plt.show()


W = 256 # wall weight
#13x13 matrix 
numRows = 13
numCols = 13

Map =   [    [  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  W,  W,  W,  W,  W,  W,  W,  1,  1,  W],
             [  W,  1,  1,  W,  1,  1,  1,  1,  1,  W,  1,  1,  W],
             [  W,  1,  1,  W,  1,  1,  1,  1,  1,  W,  1,  1,  W],
             [  W,  1,  1,  W,  1,  1,  1,  1,  1,  W,  1,  1,  W],
             [  W,  1,  1,  W,  1,  1,  W,  1,  1,  W,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  W,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  W,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  W,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  W,  1,  1,  1,  1,  1,  W],
             [  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W],
             ]

# 6 1, 7 2, 
def generateNodeNeighbors(row,col,arrMap):
    neighbors = {} 
    for i in range(-1, 2):
        for j in range(-1, 2):
            iRow = i + row
            jCol = j + col
            if iRow >= 0 and iRow < numRows and jCol >= 0 and jCol < numCols: # checks if within map bounds 
                if i != 0 or j != 0: #makes sure not mapping to same nod   
                    neighborName = str(iRow) + " " +  str(jCol)
                    neighbors.update({neighborName:arrMap[iRow][jCol]})
    return neighbors

graphMap = {}
for i in range(numRows):
    for j in range(numCols):
        nodeName = str(i) + " " + str(j)
        graphMap.update({nodeName:generateNodeNeighbors(i,j,Map)})

start_node = "6 1"
goal_node = "6 11"
path_to_goal = ucs(graphMap, start_node, goal_node)
if path_to_goal:
    print("Path to goal:", path_to_goal)
else:
    print("Goal not found.")

breakxy(path_to_goal)