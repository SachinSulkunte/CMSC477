from pupil_apriltags import Detector
import cv2
import numpy as np
import time
from robomaster import robot
from robomaster import camera

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


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

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
                # print("result:", res)
                pose = find_pose_from_tag(K, res)
                rot, jaco = cv2.Rodrigues(pose[1], pose[1])
                # print(rot)

                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                print(res.center.astype(np.int32)[0])
                # print(pose)
                
             
                ep_chassis = ep_robot.chassis

                x_val = res.center.astype(np.int32)[0]
                y_val = res.center.astype(np.int32)[1]
                z_val = 90

                # if(res.center.astype(np.int32)[0]<110):
                #     # Forward 0.5 meters
                #     ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.7).wait_for_completed()

                #     # Backward 0.5 meters
                #     ep_chassis.move(x=-0.5, y=0, z=0, xy_speed=0.7).wait_for_completed()

                #     # Move 0.5 meters left
                #     ep_chassis.move(x=0, y=-0.5, z=0, xy_speed=0.7).wait_for_completed()

                #     # Move 0.5 meters right
                #     ep_chassis.move(x=0, y=0.5, z=0, xy_speed=0.7).wait_for_completed()

                #     # Trun left 90 degrees
                #     ep_chassis.move(x=0, y=0, z=0.5, z_speed=45).wait_for_completed()

                #     # Trun right 90 degrees
                #     ep_chassis.move(x=0, y=0, z=-90, z_speed=45).wait_for_completed()

                #     ep_robot.close()
                curr_x = res.center.astype(np.int32)[0]
                curr_y = res.center.astype(np.int32)[1]

                goal_x = 320
                goal_y = 200

                x_val = 0.3
                y_val = 0.3
                z_val = 90
                
                if (curr_x < goal_x):
                    ep_chassis.drive_speed(x=0, y=-y_val, z=0, timeout=1)
                    time.sleep(1)
                if (curr_x > goal_x):
                    ep_chassis.drive_speed(x=0, y=y_val, z=0, timeout=1)
                    time.sleep(1)
                if (curr_x == goal_x):
                    time.sleep(1)
                if (curr_y < goal_y):
                    ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=1)
                    time.sleep(1)
                if (curr_y > goal_y):
                    ep_chassis.drive_speed(x=-x_val, y=0, z=0, timeout=1)
                    time.sleep(1)
                if (curr_y == goal_y):
                    time.sleep(1)


            cv2.imshow("img", img)
            cv2.waitKey(10)
            
            
            ## time.sleep(3)
        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)



# if __name__ == '__main__':
#     ep_robot = robot.Robot()
#     ep_robot.initialize(conn_type="ap")

#     ep_chassis = ep_robot.chassis

#     x_val = res.center.astype(np.int32)[0]
#     y_val = res.center.astype(np.int32)[1]
#     z_val = 90

#     # Forward 0.5 meters
#     ep_chassis.move(x=x_val, y=0 z=0, xy_speed=0.7).wait_for_completed()

#     # Backward 0.5 meters
#     ep_chassis.move(x=-x_val, y=0, z=0, xy_speed=0.7).wait_for_completed()

#     # Move 0.5 meters left
#     ep_chassis.move(x=0, y=-y_val, z=0, xy_speed=0.7).wait_for_completed()

#     # Move 0.5 meters right
#     ep_chassis.move(x=0, y=y_val, z=0, xy_speed=0.7).wait_for_completed()

#     # Trun left 90 degrees
#     ep_chassis.move(x=0, y=0, z=z_val, z_speed=45).wait_for_completed()

#     # Trun right 90 degrees
#     ep_chassis.move(x=0, y=0, z=-z_val, z_speed=45).wait_for_completed()

#     ep_robot.close()


