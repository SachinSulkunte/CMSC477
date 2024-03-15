import numpy as np
import queue
from robomaster import robot
from pupil_apriltags import Detector
import cv2
import numpy as np
import time
from robomaster import camera
import math
from numpy.linalg import inv

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
                rot = round(pose[1][2], 2) 

                goal_y = 0
                goal_x = 0.2

                duration = 0.5

                if res.tag_id == 32:

                    rot1 = 180
                    rot1 = math.radians(rot1)

                    T_ag = np.array([[math.cos(rot1), -math.sin(rot1), 0, 0.532],
                                [math.sin(rot1), math.cos(rot1), 0, 0.931],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

                    rot2 = rot
                    rot2 = math.radians(rot2)

                    T_ar = np.array([[math.cos(rot2), -math.sin(rot2), 0, curr_x],
                                [math.sin(rot2), math.cos(rot2), 0, curr_y],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

                    T_rg = np.matmul(T_ag, inv(T_ar))

                    xr_g = T_rg[0, 3]
                    yr_g = T_rg[1, 3]

                    # next point
                    x1_g = 0.665
                    y1_g = 1.463

                    vel_x = (x1_g - xr_g) / duration
                    vel_y = (y1_g - yr_g) / duration

                    ep_chassis.drive_speed(x=vel_x, y=vel_y, z=0, timeout=duration)
                elif res.tag_id == 38:
                    rot1 = 180
                    rot1 = math.radians(rot1)

                    T_ag = np.array([[math.cos(rot1), -math.sin(rot1), 0, 1.33],
                                [math.sin(rot1), math.cos(rot1), 0, 1.463],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

                    rot3 = rot
                    rot3 = math.radians(rot3)

                    T_ar = np.array([[math.cos(rot3), -math.sin(rot3), 0, curr_x],
                                [math.sin(rot3), math.cos(rot3), 0, curr_y],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

                    T_rg = np.matmul(T_ag, inv(T_ar))

                    xr_g = T_rg[0, 3]
                    yr_g = T_rg[1, 3]

                    # next point
                    x2_g = 1.197
                    y2_g = 0.931

                    vel_x = (x2_g - xr_g) / duration
                    vel_y = (y2_g - yr_g) / duration

                    ep_chassis.drive_speed(x=vel_x, y=vel_y, z=0, timeout=duration)

                elif res.tag_id == 44:
                    rot1 = 180
                    rot1 = math.radians(rot1)

                    T_ag = np.array([[math.cos(rot1), -math.sin(rot1), 0, 2.128],
                                [math.sin(rot1), math.cos(rot1), 0, 0.931],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

                    rot4 = rot
                    rot4 = math.radians(rot4)

                    T_ar = np.array([[math.cos(rot4), -math.sin(rot4), 0, curr_x],
                                [math.sin(rot4), math.cos(rot4), 0, curr_y],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

                    T_rg = np.matmul(T_ag, inv(T_ar))

                    xr_g = T_rg[0, 3]
                    yr_g = T_rg[1, 3]

                    # next point
                    x3_g = 1.197
                    y3_g = 1.729

                    vel_x = (x3_g - xr_g) / duration
                    vel_y = (y3_g - yr_g) / duration

                    ep_chassis.drive_speed(x=vel_x, y=vel_y, z=0, timeout=duration)

                    #****

                    rot5 = rot
                    rot5 = math.radians(rot5)

                    T_ar = np.array([[math.cos(rot5), -math.sin(rot5), 0, curr_x],
                                [math.sin(rot5), math.cos(rot5), 0, curr_y],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

                    T_rg = np.matmul(T_ag, inv(T_ar))

                    xr_g = T_rg[0, 3]
                    yr_g = T_rg[1, 3]

                    # next point
                    x4_g = 2.261
                    y4_g = 1.463

                    vel_x = (x4_g - xr_g) / duration
                    vel_y = (y4_g - yr_g) / duration

                    ep_chassis.drive_speed(x=vel_x, y=vel_y, z=rot+180, timeout=duration)

                elif res.tag_id == 39:
                    rot1 = 0
                    rot1 = math.radians(rot1)

                    T_ag = np.array([[math.cos(rot1), -math.sin(rot1), 0, 1.596],
                                [math.sin(rot1), math.cos(rot1), 0, 1.463],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

                    rot6 = rot
                    rot6 = math.radians(rot6)

                    T_ar = np.array([[math.cos(rot4), -math.sin(rot6), 0, curr_x],
                                [math.sin(rot6), math.cos(rot6), 0, curr_y],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

                    T_rg = np.matmul(T_ag, inv(T_ar))

                    xr_g = T_rg[0, 3]
                    yr_g = T_rg[1, 3]

                    # next point
                    x5_g = 2.793
                    y5_g = 0.931

                    vel_x = (x5_g - xr_g) / duration
                    vel_y = (y5_g - yr_g) / duration

                    ep_chassis.drive_speed(x=vel_x, y=vel_y, z=0, timeout=duration)

            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)
    





    