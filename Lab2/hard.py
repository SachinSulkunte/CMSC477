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

if __name__ == '__main__':
    print("start")
    model = YOLO('/Users/sachin/Documents/Coursework/CMSC477/CMSC477/Lab2/best_v2.pt')

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
            
            # open gripper, drive to lego
            ep_gripper.open(power=50)
            time.sleep(1)
            ep_chassis.move(x=0, y=0.0, z=15, z_speed=15).wait_for_completed()
            ep_chassis.move(x=0.3, y=0.0, z=0, xy_speed=0.7).wait_for_completed()
            time.sleep(1)
            ep_gripper.pause()

            # adjust arm position, close gripper on lego
            ep_arm.move(x=10, y=10).wait_for_completed()
            ep_gripper.close(power=50)
            time.sleep(1)
            ep_gripper.pause()

            # lift arm
            ep_arm.move(x=0, y=50).wait_for_completed()
            # drive to blue line
            ep_chassis.move(x=0.6, y=0.3, z=0, xy_speed=0.5).wait_for_completed()
            ep_chassis.move(x=0, y=0, z=-15, z_speed=0.5).wait_for_completed()

            # release lego once other robot has arrived
            ep_gripper.open(power=50)
            time.sleep(1)
            ep_gripper.pause()
            ep_robot.close()

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)

    





    
