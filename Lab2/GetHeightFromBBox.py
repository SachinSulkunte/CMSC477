from ultralytics import YOLO
import cv2
import time
from robomaster import robot
from robomaster import camera
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