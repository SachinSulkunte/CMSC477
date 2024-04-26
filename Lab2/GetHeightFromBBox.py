from ultralytics import YOLO
import cv2
import time
from robomaster import robot
from robomaster import camera
import numpy as np

distanceSignal = []

def getDistanceFromBB():
    model = YOLO('C:\\Users\\Melin\\OneDrive\\Spring 24\\CMSC477\\Lab 0\\Lab2\\better.pt')
    # Use vid instead of ep_camera to use your laptop's webcam
    # vid = cv2.VideoCapture(0)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    while True:
        # ret, frame = vid.read()
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        if frame is not None:
            if model.predictor:
                model.predictor.args.verbose = False
        results = model.predict(source=frame, show=True)

        boxes = results[0].boxes.xywh.cpu()
        for box in boxes:
            x, y, w, h = box
            print("Width of Box: {}, Height of Box: {}".format(w, h))
            print("Distance: " + str(distance_from_box_size(w, h)))

import numpy as np

distanceSignal = []

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
    distanceSignal.append(h_distance)

    #rolling median filter
    if(len(distanceSignal) > 3):
        h_distance =  np.median(distanceSignal[-3:])

    return h_distance

getDistanceFromBB()