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
from ultralytics import YOLO 
import math

# (1) compute own robot's distance and orientation w/respect to the lego - distance_from_box_size()
# (2) move the own robot to toward the lego so that the distance to the lego is 10cm, a distance that robot's arm can reach 
# (3) open gripper
# (4) extend the own robot's arm (10cm)
# (5) close

distanceSignal = []

def find_pose_from_object(K, x, y, w, h, box):
    w_half_size = w / 2
    h_half_size = h / 2

    marker_center = np.array((0, 0, 0))
    marker_points = []
    marker_points.append(marker_center + (-w_half_size, h_half_size, 0))
    marker_points.append(marker_center + ( w_half_size, h_half_size, 0))
    marker_points.append(marker_center + ( w_half_size, -h_half_size, 0))
    marker_points.append(marker_center + (-w_half_size, -h_half_size, 0))
    _marker_points = np.array(marker_points)

    object_points = _marker_points
    image_points = box.corners

    pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if pnp_ret[0] == False:
        raise Exception('Error solving PnP')

    r = pnp_ret[1]
    p = pnp_ret[2]

    return p.reshape((3,)), r.reshape((3,))

def distance_from_box_size(boxWidth, boxHeight):
    #param
    a = 80.8847
    b = -214.7402
    c = 186.6250
    expected_aspect_ratio = 7 #h/w

    actual_aspect_ratio = boxHeight / boxWidth
    correction = expected_aspect_ratio/actual_aspect_ratio
    actualHeight = boxHeight * correction
    
    h_distance = a * actualHeight^2 + b * actualHeight + c 
    distanceSignal.append(h_distance)

    #rolling median filter
    if(len(distanceSignal) >= 3):
        h_distance =  np.median(distanceSignal[-3:])

    return h_distance


if __name__ == '__main__':
    model = YOLO('C:\\Users\\Melin\\OneDrive\\Spring 24\\CMSC477\\Lab 0\\Lab2\\lego.pt')
    # Use vid instead of ep_camera to use your laptop's webcam
    # vid = cv2.VideoCapture(0)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    while True:
        try:
            # ret, frame = vid.read()
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            if frame is not None:
                if model.predictor:
                    model.predictor.args.verbose = False
            results = model.predict(source=frame, show=True)

            boxes = results[0].boxes.xywh.cpu()
            # Extract bounding boxes, classes, names, and confidences
            classes = results[0].boxes.cls.tolist()
            names = results[0].names
            confidences = results[0].boxes.conf.tolist()

            # Iterate through the results
            for box, cls, conf in zip(boxes, classes, confidences):
                x, y, w, h = box
                confidence = conf
                detected_class = cls
                name = names[int(cls)]
                
                print("x: {}, y: {}".format(x, y))
                print("Width of Box: {}, Height of Box: {}".format(w, h))
                print("Distance: " + str(distance_from_box_size(w, h)))
                print("Detected object: ", name)

                if name == 'lego':
                    pose = find_pose_from_object(K, x, y, w, h, box)
                    rot, jaco = cv2.Rodrigues(pose[1], pose[1])

                    pts = box.corners.reshape((-1, 1, 2)).astype(np.int32)
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                    cv2.circle(img, tuple(box.center.astype(np.int32)), 5, (0, 0, 255), -1)
                
                    rot = round(pose[1][2], 2) 
                    
                    duration = 0.5
                    goal_x = 10.0 # 10cm away from the lego object
                    goal_y = 10.0 # 10cm away from the lego object
                    
                    vel_x = (distance_from_box_size(w, h) - goal_x) * math.cos(rot) / duration
                    vel_y = (distance_from_box_size(w, h) - goal_y) * math.cos(rot) / duration
                    ep_chassis.drive_speed(x=vel_x, y=vel_y, z=rot, timeout=duration)

                    ep_gripper.open(power=50)
                    time.sleep(1)
                    ep_gripper.pause()

                    ep_arm.move(x=goal_x - 1.0, y=goal_y - 1.0).wait_for_completed()

                    ep_gripper.close(power=50)
                    time.sleep(1)
                    ep_gripper.pause()

                    ep_robot.close()

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)

    





    