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

import detect_blue_line

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

distanceSignal = []

# def getDistanceFromBB():
#     model = YOLO('C:\\Users\\Melin\\OneDrive\\Spring 24\\CMSC477\\Lab 0\\Lab2\\better.pt')
#     # Use vid instead of ep_camera to use your laptop's webcam
#     # vid = cv2.VideoCapture(0)
#     ep_robot = robot.Robot()
#     ep_robot.initialize(conn_type="ap")
#     ep_camera = ep_robot.camera
#     ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
#     while True:
#         # ret, frame = vid.read()
#         frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
#         if frame is not None:
#             if model.predictor:
#                 model.predictor.args.verbose = False
#         results = model.predict(source=frame, show=True)

#         boxes = results[0].boxes.xywh.cpu()
#         for box in boxes:
#             x, y, w, h = box
#             print("Width of Box: {}, Height of Box: {}".format(w, h))
#             print("Distance: " + str(distance_from_box_size(w, h)))

def distance_from_box_size(boxWidth, boxHeight):

    #param
    a = -6.79187460413004e-07
    b = 2.963889750447219e-04
    c = -0.0469580266109994
    d = 3.02446421107312
    vertexHeight = 185 #closest distance quad fit is modelled for

    print("actualHeight: " + str(boxHeight))
    if boxHeight > vertexHeight:
        h_distance = 0.1
    else:
        h_distance = a * np.power(boxHeight,3) + b * np.power(boxHeight,2) + c * boxHeight + d
    # distanceSignal.append(h_distance)

    #rolling median filter
    # if(len(distanceSignal) > 3):
    #     h_distance =  np.median(distanceSignal[-3:])

    return h_distance

def grab_frame(camera):
    ret, frame = camera.read()
    return frame

def snip_image(img):

    snip = img[(img.shape[0] - 300):(img.shape[0] - 50), (0):(img.shape[1]-100)]
    return snip

def thres_image(img):

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


    # For tracking orange
    # orangeLower = np.array([0, 103, 220]) 
    # orangeUpper = np.array([179, 255, 255]) 
    # orange_mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    # cv2.imshow("hsv orange mask", orange_mask)
    # thresholded = cv2.bitwise_and(img,img, mask=orange_mask)

    # For tracking blue
    blueLower = np.array([96,147,59])
    blueUpper = np.array([179,255,255])
    blue_mask = cv2.inRange(hsv, blueLower, blueUpper)
    #cv2.imshow("hsv",hsv)
    #cv2.imshow("hsv blue mask", blue_mask)
    thresholded = cv2.bitwise_and(img,img, mask=blue_mask)

    return thresholded

def blur_img(img):
    return cv2.GaussianBlur(img,(15,15),0)

def edge_img(img):
    return cv2.Canny(img, 30, 50)

def line_image(img):
    return cv2.HoughLines(img, 1, np.pi/180, 20)

def plot_Hough_Lines(img, rho, theta):
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    x2 = int(x0 - 1000*(-b))
    y1 = int(y0 + 1000*a)
    y2 = int(y0 - 1000*a)
    cv2.line(img, (x1,y1), (x2,y2), (0, 0, 255), 5)

    a1 = np.cos(theta-theta)
    b1 = np.sin(theta-theta)
    x01 = a*rho
    y01 = b*rho
    x11 = int(x01 + 1000*(-b1))
    x21 = int(x01 - 1000*(-b1))
    y11 = int(y01 + 1000*a1)
    y21 = int(y01 - 1000*a1)

    cv2.line(img, (x11+272,y11), (x21+272,y21), (0, 255, 0), 5)
    return img

def getAngle():

    image = grab_frame("Insert the name of the camera feed from here?")
    
    # image = cv2.imread("robot_blorange.jpg")

    # cv2.imshow("Caption",image)

    snip = snip_image(image)
    # cv2.imshow("Snip",snip)

    thresholded= thres_image(snip)
    # cv2.imshow("Thresholded Snip",thresholded)

    blurred = blur_img(thresholded)
    # cv2.imshow("Blurred Image", blurred)

    edged = edge_img(blurred)
    # cv2.imshow("Edged", edged)

    lines = line_image(edged)
    print(lines)

    if lines != None:
    
        rho = []
        theta = []
        for i in range(0, len(lines)):
            for r, o in lines[i]:
                rho.append(r)
                theta.append(o)
        print(rho)
        print(theta)
        print("---")
        rho_avg = np.mean(rho)
        theta_avg = np.mean(theta)
        angle = (180/np.pi)*theta_avg

        final = plot_Hough_Lines(snip,rho_avg,theta_avg)
        cv2.imshow("Final",final)

        print(angle)




        print("---")
        cv2.waitKey(0)

        return angle



if __name__ == '__main__':
    print("start")
    model = YOLO('/Users/sachin/Documents/Coursework/CMSC477/CMSC477/Lab2/best_v2.pt')
    # Use vid instead of ep_camera to use your laptop's webcam
    # vid = cv2.VideoCapture(0)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    print("connected")
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

            stage1 = False

            if stage1 is False:
                # Iterate through the results
                for box, cls, conf in zip(boxes, classes, confidences):
                    x, y, w, h = box
                    K=np.array([[184.752, 0, 320], [0, 184.752, 180], [0, 0, 1]])
                    confidence = conf
                    detected_class = cls
                    name = names[int(cls)]

                    if name == 'lego':
                        print("x: {}, y: {}".format(x, y))
                        print("Width of Box: {}, Height of Box: {}".format(w, h))
                        print("Distance: " + str(distance_from_box_size(w, h)))
                        print("Detected object: ", name)
                        
                        # pose = find_pose_from_object(K, x, y, w, h, box)
                        # rot, jaco = cv2.Rodrigues(pose[1], pose[1])

                        # pts = box.corners.reshape((-1, 1, 2)).astype(np.int32)
                        # img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                        # cv2.circle(img, tuple(box.center.astype(np.int32)), 5, (0, 0, 255), -1)
                    
                        # rot = round(pose[1][2], 2) 
                        
                        duration = 0.5
                        goal_x = 10.0 # 10cm away from the lego object
                        goal_y = 10.0 # 10cm away from the lego object
                        
                        dist = float(distance_from_box_size(w, h)) - 0.32

                        # vel_x = (distance_from_box_size(w, h) - goal_x)  / duration
                        # vel_y = (distance_from_box_size(w, h) - goal_y) / duration
                        # print("vel")
                        # print(vel_x, vel_y)
                        # ep_chassis.drive_speed(x=vel_x, y=vel_y, z=0, timeout=duration)
                        
                        # open gripper, drive to lego
                        ep_gripper.open(power=50)
                        time.sleep(1)
                        ep_chassis.move(x=dist, y=0, z=0, xy_speed=0.7).wait_for_completed()
                        time.sleep(1)
                        ep_gripper.pause()

                        # adjust arm position, close gripper on lego
                        ep_arm.move(x=goal_x - 1.0, y=goal_y - 1.0).wait_for_completed()
                        ep_gripper.close(power=50)
                        time.sleep(1)
                        ep_gripper.pause()

                        # lift arm
                        ep_arm.move(x=0, y=50).wait_for_completed()
                        stage1 = True # move on to finding blue line
            else:
                # drive to blue line
                angle, is_close = detect_blue_line(frame)
                ep_chassis.move(x=0, y=0, z=-angle, z_speed=20).wait_for_completed() # rotate to blue line
                
                if not is_close: # drive forward if not close yet
                    ep_chassis.drive_speed(x=0.3, y=0, z=0, timeout=1)
                    time.sleep(0.5)
                else:
                    # release lego once other robot has arrived
                    time.sleep(5)
                    ep_gripper.open(power=50)
                    time.sleep(1)
                    ep_gripper.pause()
                    ep_robot.close()
                    break

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)

    





    