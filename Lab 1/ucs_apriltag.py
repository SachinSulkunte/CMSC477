import numpy as np
import queue

obstacles = ['0 2', '0 3', '0 4', '0 5', '0 6', '0 7', '0 8', '1 2', '1 8', '2 2', '2 8', '3 2', '3 8', '4 2', '4 8', '4 5', '5 5', '6 5', '7 5', '8 5', '9 5', '10 5']

def generateNodeNeighbors(row,col,arrMap):
    neighbors = {} 
    for i in range(-1, 2):
        for j in range(-1, 2):
            iRow = i + row
            jCol = j + col
            #print(i, j, row, col, iRow, jCol)
            if iRow >= 0 and iRow < matrix.shape[0] and jCol >= 0 and jCol < matrix.shape[1]: # checks if within map bounds 
                if i != 0 or j != 0: #makes sure not mapping to same nod   
                    neighborName = str(iRow) + " " +  str(jCol)
                    if neighborName in obstacles:
                        neighbors.update({neighborName:1000.0})
                    else:
                        if (i == 1 and j == 0) or (i == -1 and j == 0) or (j == 1 and i == 0) or (j == -1 and i == 0):
                            #print(row, col, iRow, jCol)
                            neighbors.update({neighborName:26.6})
                        else:
                            neighbors.update({neighborName:37.618})
    return neighbors

def ucs(graph, start, goal):
    visited = set()
    frontier = queue.PriorityQueue()  # Using a priority queue
    frontier.put((0, start, [start]))  # Adding start node with cost 0

    while not frontier.empty():
        cost, node, path = frontier.get()
        if node not in visited:
            visited.add(node)
            # Do something with the node if needed

            if node == goal:
                return path  # Return the path to the goal

            # Explore neighbors
            for neighbor, neighbor_cost in graph[node].items():
                if neighbor not in visited:
                    new_cost = cost + neighbor_cost
                    frontier.put((new_cost, neighbor, path + [neighbor]))  # Add to the priority queue

    return None  # Goal not found

graphMap = {}
matrix = np.ones((11, 11))
for iy, ix in np.ndindex(matrix.shape):
    nodeName = str(iy) + " " + str(ix)
    graphMap.update({nodeName:generateNodeNeighbors(iy,ix,matrix)})

start_node = '3 0'
goal_node = '3 10'
path_to_goal = ucs(graphMap, start_node, goal_node)
if path_to_goal:
    print("Path to goal:", path_to_goal)
else:
    print("Goal not found.")
