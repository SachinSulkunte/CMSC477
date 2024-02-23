import queue

def ucs(graph, start, goal):
    visited = set()
    frontier = queue.PriorityQueue()  # Using a priority queue
    frontier.put((0, start, [start]))  # Adding start node with cost 0

    while not frontier.empty():
        cost, node, path = frontier.get()
        if node not in visited:
            visited.add(node)
            # Do something with the node if needed
            #print(node)

            if node == goal:
                return path  # Return the path to the goal

            # Explore neighbors
            for neighbor, neighbor_cost in graph[node].items():
                if neighbor not in visited:
                    new_cost = cost + neighbor_cost
                    frontier.put((new_cost, neighbor, path + [neighbor]))  # Add to the priority queue

    return None  # Goal not found


#start new code
W = 256 # wall weight
#13x13 matrix 
numRows = 26
numCols = 26 

oldMap =   [ [  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W],
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

Map =       [[  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  W,  W,  W,  W,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  W,  W,  W,  W,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  W,  W,  W,  W,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  W,  W,  W,  W,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W,  W,  W,  W,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  W],
             [  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W,  W],
             ]
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
#print(graphMap)
#end new code

start_node = "12 4"
goal_node = "12 22"
path_to_goal = ucs(graphMap, start_node, goal_node)
if path_to_goal:
    print("Path to goal:", path_to_goal)
else:
    print("Goal not found.")
