import queue

def add(adj_list, a, b):
    adj_list.setdefault(a, []).append(b)
    adj_list.setdefault(b, []).append(a)

def get_adjMatrix(matrix):
    adj_list = {}
    print(f"len(matrix[3]), {len(matrix[3])}")
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if j < len(matrix[i]) - 1:
                add(adj_list, matrix[i][j], matrix[i][j+1])
            if i < len(matrix[i]) - 1:
                for x in range(max(0, j - 1), min(len(matrix[i+1]), j+2)):
                    add(adj_list, matrix[i][j], matrix[i+1][x])
    return adj_list

def get_costDict(matrix, obs):
    cost_dict = {}
    for key, val in matrix.items():
        cell_dict = {}
        for v in val:
            if v in obs:
                cell_dict[v] = 100
            else:
                cell_dict[v] = 1
        cost_dict[key] = cell_dict
    return cost_dict

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

matrix = [
    ['00', '01', '02', '03', '04', '05', '06', '07', '08', '09', '010'],
    ['10', '11', '12', '13', '14', '15', '16', '17', '18', '19', '110'],
    ['20', '21', '22', '23', '24', '25', '26', '27', '28', '29', '210'],
    ['30', '31', '32', '33', '34', '35', '36', '37', '38', '39', '310'],
    ['40', '41', '42', '43', '44', '45', '46', '47', '48', '49', '410'],
    ['50', '51', '52', '53', '54', '55', '56', '57', '58', '59', '510'],
    ['60', '61', '62', '63', '64', '65', '66', '67', '68', '69', '610'],
    ['70', '71', '72', '73', '74', '75', '76', '77', '78', '79', '710'],
    ['80', '81', '82', '83', '84', '85', '86', '87', '88', '89', '810'],
    ['90', '91', '92', '93', '94', '95', '96', '97', '98', '99', '1010'],
    ['100', '101', '102', '103', '104', '105', '106', '107', '108', '109', '1010']
]

obstacles = ['02', '03', '04', '05', '06', '07', '08', '12', '18', '22', '28', '32', '38', '42', '48', '45', '55', '65', '75', '85', '95', '105']

graph = get_costDict(get_adjMatrix(matrix), obstacles)

start_node = '30'
goal_node = '310'
path_to_goal = ucs(graph, start_node, goal_node)
if path_to_goal:
    print("Path to goal:", path_to_goal)
else:
    print("Goal not found.")