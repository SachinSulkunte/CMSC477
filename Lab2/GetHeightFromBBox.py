from ultralytics import YOLO
import cv2
import time
from robomaster import robot
from robomaster import camera
import numpy as np

distanceSignal = []

def getDistanceFromBB():
    model = YOLO('C:\\Users\\Melin\\OneDrive\\Spring 24\\CMSC477\\Lab 0\\Lab2\\lego.pt')
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
    a = 80.8847
    b = -214.7402
    c = 186.6250
    expected_aspect_ratio = 7 #h/w
    vertexHeight = 1.22613 #closest distance quad fit is modelled for

    actual_aspect_ratio = boxHeight / boxWidth
    correction = expected_aspect_ratio/actual_aspect_ratio
    actualHeight = boxHeight * correction
    if actualHeight >  vertexHeight:
        h_distance = 45
    else:
        h_distance = a * actualHeight^2 + b * actualHeight + c 
    distanceSignal.append(h_distance)

    #rolling median filter
    if(len(distanceSignal) > 3):
        h_distance =  np.median(distanceSignal[-3:])

    return h_distance

getDistanceFromBB()