from pupil_apriltags import Detector
import cv2
import numpy as np
import time
from robomaster import robot
from robomaster import camera
# import aprilTagPathing

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

def change_direction(stage, path, current_ind):

    x = path[current_ind].split(' ')[0]
    y = path[current_ind].split(' ')[1]

    next_x = path[current_ind + 1].split(' ')[0]
    next_y = path[current_ind + 1].split(' ')[1]

    if x < next_x:
        y_vel = 0.3
        x_vel = 0
    else:
        y_vel = -0.3
        x_vel = 0
    
    return stage

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis

    # path = aprilTagPathing.compute_path()

    tag_size=0.16 # tag size in meters

    x_val = 0.0
    y_val = 0.3
    z_val = 0
    
    stage = 1 # tracking coordinate step through map

    ep_chassis.drive_speed(x=x_val, y=y_val, z=z_val, timeout=5)
    while(1):
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)  
            cv2.imwrite("/home/user/Desktop/test.png", img) 
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)

            K=np.array([[184.752, 0, 320], [0, 184.752, 180], [0, 0, 1]])
            results = at_detector.detect(gray, estimate_tag_pose=False)
            for res in results:
                if res.tag_id == 38 and stage == 1:
                    pose = find_pose_from_tag(K, res)
                    rot, jaco = cv2.Rodrigues(pose[1], pose[1])

                    pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                    cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)

                    curr_y = round(pose[0][0],2)
                    curr_x = round(pose[0][2], 2)
                    rot = round(pose[1][2], 2) * 1000 / 3.14
                    
                    goal_y = -0.05
                    goal_x = 0.17
                    duration = 0.75
        
                    vel_x = (curr_x - goal_x) / duration
                    vel_y = (curr_y - goal_y) / duration

                    ep_chassis.drive_speed(x=vel_x, y=vel_y, z=0, timeout=duration)

                    if abs(curr_y - goal_y) < 0.05 and abs(curr_x -  goal_x) < 0.05:
                        print("moving next stage")
                        stage = 2
                        ep_chassis.drive_speed(x=x_val, y=-y_val, z=z_val, timeout=5)
                
                elif res.tag_id == 44 and stage == 2:
                    pose = find_pose_from_tag(K, res)
                    rot, jaco = cv2.Rodrigues(pose[1], pose[1])

                    pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                    cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)

                    curr_y = round(pose[0][0],2)
                    curr_x = round(pose[0][2], 2)
                    rot = round(pose[1][2], 2) * 1000 / 3.14
                    
                    goal_y = 0.05
                    goal_x = 0.17
                    duration = 0.75
        
                    vel_x = (curr_x - goal_x) / duration
                    vel_y = (curr_y - goal_y) / duration

                    ep_chassis.drive_speed(x=vel_x, y=vel_y, z=0, timeout=duration)

                    if abs(curr_y - goal_y) < 0.05 and abs(curr_x -  goal_x) < 0.05:
                        stage = 3
                        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=1) # reset speed
                        time.sleep(1)
                        print("start spin")
                        ep_chassis.move(x=0, y=0, z=-180, z_speed=45).wait_for_completed()
                        print("spinning")
                        time.sleep(1)
                        ep_chassis.move(x=-0.1, y=0, z=0).wait_for_completed()

                        ep_chassis.drive_speed(x=0, y=-y_val, z=0, timeout=5) #call to get direction
                        time.sleep(1)
                elif res.tag_id == 39 and stage == 3:
                    pose = find_pose_from_tag(K, res)
                    rot, jaco = cv2.Rodrigues(pose[1], pose[1])

                    pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                    cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)

                    curr_y = round(pose[0][0],2)
                    curr_x = round(pose[0][2], 2)
                    rot = round(pose[1][2], 2) * 1000 / 3.14
                    
                    goal_y = 0.1
                    goal_x = 0.8 # clear corner
                    duration = 1
        
                    vel_x = (curr_x - goal_x) / duration
                    vel_y = (curr_y - goal_y) / duration

                    ep_chassis.drive_speed(x=vel_x, y=vel_y, z=0, timeout=duration)

                    if abs(curr_y - goal_y) < 0.05 and abs(curr_x -  goal_x) < 0.05:
                        stage = 4
                        ep_chassis.drive_speed(x=0, y=0.3, z=0, timeout=5)

                elif res.tag_id == 45 and stage == 4:
                    pose = find_pose_from_tag(K, res)
                    rot, jaco = cv2.Rodrigues(pose[1], pose[1])

                    pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                    cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)

                    curr_y = round(pose[0][0],2)
                    curr_x = round(pose[0][2], 2)
                    rot = round(pose[1][2], 2) * 1000 / 3.14
                    
                    goal_y = 0.00
                    goal_x = 0.26  # align on goal square
                    duration = 1
        
                    vel_x = (curr_x - goal_x) / duration
                    vel_y = (curr_y - goal_y) / duration

                    ep_chassis.drive_speed(x=vel_x, y=vel_y, z=0, timeout=duration)

                    if abs(curr_y - goal_y) < 0.05 and abs(curr_x -  goal_x) < 0.05:
                        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop movement
                        break
                    
            cv2.imshow("img", img)
            cv2.waitKey(6)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)
    
    ep_robot.close()
