import queue
import matplotlib.pyplot as plt

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


#start new code
W = 256 # wall weight
#13x13 matrix 
numRows = 10
numCols = 13 

Map =   [    [  W,  1,  1,  W,  W,  W,  W,  W,  W,  W,  1,  1,  W],
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

def generateNodeNeighbors(row, col, arrMap):
    neighbors = {}
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Cardinal directions: right, left, down, up
    for dir_row, dir_col in directions:
        iRow = row + dir_row
        jCol = col + dir_col
        if 0 <= iRow < numRows and 0 <= jCol < numCols:  # Check if within map bounds
            neighborName = str(iRow) + " " + str(jCol)
            neighbors[neighborName] = arrMap[iRow][jCol]
    return neighbors

# takes in boolean 
def compute_path():
    graphMap = {}
    for i in range(numRows):
        for j in range(numCols):
            nodeName = str(i) + " " + str(j)
            graphMap.update({nodeName:generateNodeNeighbors(i,j,Map)})

    start_node = "3 1"
    goal_node = "3 11"
    path_to_goal = ucs(graphMap, start_node, goal_node)
    if path_to_goal:
        return path_to_goal
    else:
        return None

def visualize_path(coordinates):
    x_coords = []
    y_coords = []
    for coord in coordinates:
        x, y = coord.split()
        x_coords.append(int(x))
        y_coords.append(int(y))
    
    # Set up the grid
    plt.figure()
    plt.grid(True)
        
    # Plot the path
    plt.plot(x_coords, y_coords, marker='o')
    
    # Set plot limits
    plt.xlim(min(x_coords) - 1, max(x_coords) + 1)
    plt.ylim(min(y_coords) - 1, max(y_coords) + 1)
    
    # Switch axis labels
    plt.xlabel('Y')
    plt.ylabel('X')
    plt.title('Path')
    # Show plot
    plt.show()

path = compute_path()
visualize_path(path)
