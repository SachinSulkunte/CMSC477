import numpy as np
import queue
from robomaster import robot
from pupil_apriltags import Detector
import cv2
import numpy as np
import time
from robomaster import camera

obstacles = ['0 2', '0 3', '0 4', '0 5', '0 6', '0 7', '0 8', '1 2', '1 8', '2 2', '2 8', '3 2', '3 8', '4 2', '4 8', '4 5', '5 5', '6 5', '7 5', '8 5', '9 5', '10 5']
XYdistanceDict = {}

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

def find_pose_from_tag(K, detection):
    m_half_size = tag_size / 2

    marker_center = np.array((0, 0, 0))
    marker_points = []
    marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
    marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
    _marker_points = np.array(marker_points)

    object_points = _marker_points
    image_points = detection.corners

    pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if pnp_ret[0] == False:
        raise Exception('Error solving PnP')

    r = pnp_ret[1]
    p = pnp_ret[2]

    return p.reshape((3,)), r.reshape((3,))

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

def populateXYdistanceDict(path_to_goal):
    for i in range(len(path_to_goal)-1):
        edgeCost = graphMap[path_to_goal[i]][path_to_goal[i+1]]
        nodeOne = path_to_goal[i].split(' ')
        nodeTwo = path_to_goal[i+1].split(' ')
        if edgeCost == 26.6:
            if int(nodeOne[0]) == int(nodeTwo[0]) and int(nodeOne[1])+1 == int(nodeTwo[1]):
                x_delta = 0.266
                y_delta = 0
        elif edgeCost == 37.618:
            if int(nodeOne[0])+1 == int(nodeTwo[0]) and int(nodeOne[1])+1 == int(nodeTwo[1]):
                x_delta = 0.266
                y_delta = 0.266
            elif int(nodeOne[0])-1 == int(nodeTwo[0]) and int(nodeOne[1])+1 == int(nodeTwo[1]):
                x_delta = 0.266
                y_delta = -0.266
        key = path_to_goal[i] + ' ' + path_to_goal[i+1]
        XYdistanceDict[key] = [x_delta, y_delta]

if __name__ == '__main__':
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

    populateXYdistanceDict(path_to_goal)

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    ep_chassis = ep_robot.chassis

    tag_size=0.16 # tag size in meters

    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)  
            cv2.imwrite("/home/user/Desktop/test.png", img) 
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)

            K=np.array([[184.752, 0, 320], [0, 184.752, 180], [0, 0, 1]])
            results = at_detector.detect(gray, estimate_tag_pose=False)

            for res in results:
                pose = find_pose_from_tag(K, res)
                rot, jaco = cv2.Rodrigues(pose[1], pose[1])

                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
            
                curr_y = round(pose[0][0], 2)
                curr_x = round(pose[0][2], 2)

                if res.tag_id == 32:
                    x_dist = XYdistanceDict['3 0 4 1'][0] + XYdistanceDict['4 1 5 2'][0]
                    y_dist = XYdistanceDict['3 0 4 1'][1] + XYdistanceDict['4 1 5 2'][1]
                    x_avg = (curr_x + x_dist) / 2.0
                    y_avg = (curr_y + y_dist) / 2.0
                    ep_chassis.move(x=x_dist, y=y_dist, z=0, xy_speed=0.7).wait_for_completed()
                elif res.tag_id == 38:
                    x_dist = XYdistanceDict['5 2 4 3'][0] + XYdistanceDict['4 3 3 4'][0]
                    y_dist = XYdistanceDict['5 2 4 3'][1] + XYdistanceDict['4 3 3 4'][1]
                    x_avg = (curr_x + x_dist) / 2.0
                    y_avg = (curr_y + y_dist) / 2.0
                    ep_chassis.move(x=x_dist, y=y_dist, z=0, xy_speed=0.7).wait_for_completed()
                elif res.tag_id == 44:
                    x_dist = XYdistanceDict['3 4 3 5'][0] + XYdistanceDict['3 5 3 6'][0]
                    y_dist = XYdistanceDict['3 4 3 5'][1] + XYdistanceDict['3 5 3 6'][1]
                    x_avg = (curr_x + x_dist) / 2.0
                    y_avg = (curr_y + y_dist) / 2.0
                    ep_chassis.move(x=x_dist, y=y_dist, z=0, xy_speed=0.7).wait_for_completed()
                elif res.tag_id == 44:
                    x_dist = XYdistanceDict['3 6 4 7'][0] + XYdistanceDict['4 7 5 8'][0]
                    y_dist = XYdistanceDict['3 6 4 7'][1] + XYdistanceDict['4 7 5 8'][1]
                    x_avg = (curr_x + x_dist) / 2.0
                    y_avg = (curr_y + y_dist) / 2.0
                    ep_chassis.move(x=x_dist, y=y_dist, z=180, xy_speed=0.7).wait_for_completed()
                elif res.tag_id == 39:
                    x_dist = -1.0 * (XYdistanceDict['5 8 4 9'][0] + XYdistanceDict['4 9 3 10'][0])
                    y_dist = -1.0 * (XYdistanceDict['5 8 4 9'][1] + XYdistanceDict['4 9 3 10'][1])
                    x_avg = (curr_x + x_dist) / 2.0
                    y_avg = (curr_y + y_dist) / 2.0
                    ep_chassis.move(x=x_dist, y=y_dist, z=0, xy_speed=0.7).wait_for_completed()

            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)
    





    