from ultralytics import YOLO
import cv2 
import numpy as np
import time
from robomaster import robot
from robomaster import camera
import matplotlib.pyplot as plt

def find_blue(image):
    original_image = image.copy()  # preserve original img
    img = np.array(image)
    
    # blue color range
    lower_blue = np.array([70,68,90])
    upper_blue = np.array([200,255,255])
    cc = 0 # snip bounds
    aa = 100
    snip = np.zeros((img.shape[0], img.shape[1]), dtype ="uint8")
    snip = img[(img.shape[0] - aa):(img.shape[0] - cc), (0):(img.shape[1])]

    hsv = cv2.cvtColor(snip, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # image contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        longest_contour = None
        max_contour_length = 0
    
        # find longest contour
        longest_contour = contours[0]
        for contour in contours:
            contour_length = cv2.arcLength(contour, True)
            
            if contour_length > max_contour_length:
                max_contour_length = contour_length
                longest_contour = contour

        vx, vy, x, y = cv2.fitLine(longest_contour, cv2.DIST_L2, 0, 0.01, 0.01)
        contour_length = cv2.arcLength(longest_contour, True)
        # determine line slope
        slope = vy / vx
        
        # draw line
        left_point = (int(x - 1000 * vx), int(y - 1000 * vy))
        right_point = (int(x + 1000 * vx), int(y + 1000 * vy))
        line_image = cv2.line(snip, left_point, right_point, (0, 255, 0), 2)
        
        #cv2.imshow("Mask",mask)
        cv2.imshow("Line",line_image)
        cv2.waitKey(10)
        
        # return angle, line position
        return slope[0], y[0], contour_length
    
    else:
        print("No blue line found in the image.")
        return 0, 0, 0

# position handler
def sub_data_handler(sub_info):
    pos_x, pos_y = sub_info
    print("Robotic Arm: pos x:{0}, pos y:{1}".format(pos_x, pos_y))
    global XP
    global YP
    XP = pos_x
    YP = pos_y


# smooth data using moving average
def moving_avg(y_data, pt):
    data = [None] * len(y_data)
    for i in range(len(y_data)):
        avg = 0
        count = 0
        for j in range(pt):
            if i - j >= 0:
                avg += y_data[i-j]
                count += 1

        data[i] = avg/count
    return data[-1]

# calc distance from bounding box
def box_distance(dist, w, h):
    # random constants
    # m - Z and b - K
    Zw = 100
    Kw = 0
    Zh = 100
    Kh = 0
    w_distance = Zw/w + Kw
    h_distance = Zh/h + Kh
    distance = (w_distance + h_distance)/2
    dist.append(distance)
    pts = 3 # for moving avg calculation
    current_distance = moving_avg(dist, pts)
    return current_distance

def resetArmPos():
    # open gripper
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    
    # move arm
    ep_arm.sub_position(freq=5, callback=sub_data_handler)
    time.sleep(1)
    x1 = 80
    y1 = 75
    ep_arm.moveto(x=211, y=4294967242).wait_for_completed()
    # ep_arm.moveto(y=(y1-YP)-5).wait_for_completed()
    # ep_arm.moveto(y=-10).wait_for_completed()
    time.sleep(1)
    ep_arm.unsub_position()

def findLego(model):
    # spin until lego found
    legoFound = False
    z_val = 40 # degrees to spin
    while not legoFound:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=5)  
        cv2.imshow("Frame",frame)
        cv2.waitKey(10)
        results = model.predict(source=frame, conf=0.8)

        # stop spinning once lego is found
        if len(results[0]) > 0:
            legoFound = True
        else:
            ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5)
            
    # Stop spinning
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)

def driveToLego(model):
    
    dist = []
    target_distance = 1.6 # arbitrary
    distance_tol = 0.07
    box_offset_tol = 3 # num pixels left/right of center
    ON_TARGET = False
    while(not ON_TARGET): 
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=5)  
        img = np.array(frame)
        cv2.imshow("Frame",frame)
        cv2.waitKey(10)
        results = model.predict(source=frame, conf=0.7)

        conf = results[0].boxes.conf
        conf = conf.cpu()
        if len(conf) > 0:
            best = np.argmax(conf)
        else:
            continue
        coords = results[0].boxes.xywh
        
        # find coords for bounding box w/ highest confidence
        c = coords[best]
        x_center = int(img.shape[1]/2)
        x = int(c[0])
        y = int(c[1])
        w = int(c[2])
        h = int(c[3])
        box_offset = x_center - x
        distance = box_distance(dist, w, h)
        if (abs(box_offset) > box_offset_tol) and not ON_TARGET:
            # turn till box is in the center
            Kz = -0.15
            z_val = box_offset * Kz
            ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5)
        elif (abs(distance - target_distance) >= distance_tol) and not ON_TARGET :
            # move till box is at target distance
            Kx = 0.175
            x_val = (distance - target_distance) * Kx
            # print(distance)
            ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=5)
        elif (abs(distance - target_distance) < distance_tol):
            # done movement
            ON_TARGET = True
            x_val = 0.21 # move slightly further - adjust as needed
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
            ep_chassis.move(x=x_val, y=0, z=0, xy_speed=0.7).wait_for_completed()

    cv2.destroyAllWindows()

def grabLego():
    ep_arm.sub_position(freq=5, callback=sub_data_handler)
    time.sleep(1)
    x2 = 150
    y2 = 20
    x3 = 190
    y3 = 50
    # move arm position
    ep_arm.move(x = (x2 - XP) + 5).wait_for_completed()
    ep_arm.move(y = (y2 - YP) - 5).wait_for_completed()
    ep_arm.move(x = (x3 - XP) + 5).wait_for_completed()
    
    # grip lego
    ep_gripper.close(power=100)
    time.sleep(2.5)
    ep_gripper.pause()
    
    # move arm position
    ep_arm.move(y = (y3 - YP) + 5).wait_for_completed()
    time.sleep(0.5)
    ep_arm.unsub_position()

def findLine():
    # rotate until blue line found
    lineFound = False
    z_val = 10
    min_cont_len = 300
    
    while not lineFound:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=5)  
        slope, y, cont_len = find_blue(img)
        
        # rotate if not found
        if cont_len < min_cont_len:
            ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5)
        else:
            lineFound = True
    # stop spinning
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    
def driveToLine():
    # drive closer to the blue line and straighten out
    angle_tol = 0.01
    Kz = 100
    y_Goal = 100
    y = y_Goal
    y_tol = 10
    straight = False
    done = False
    # align to blue line
    while not done:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=5)  
            slope, y, cont_size = find_blue(img)
            y_dist = y_Goal-y
            if slope == None:
                findLine()
            # rotated relative to line
            if (np.abs(slope) > angle_tol) and not straight:
                z_val = Kz * slope
                ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5)
            # flip boolean
            elif not straight: 
                ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                ep_chassis.move(x=0, y=0, z=0, xy_speed=0.7).wait_for_completed()
                straight = True
            # move distance once aligned
            elif straight and not done:
                print(y_dist)
                if abs(y_dist) > y_tol:
                    Kx = 0.005
                    x_val = Kx * y_dist
                    if (np.abs(slope) > angle_tol):
                        z_val = 0.5 * Kz * slope
                    else: 
                        z_val = 0
                    ep_chassis.drive_speed(x=x_val, y=0, z=z_val, timeout=5)
                else: 
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                    x_dist = 0.43
                    time.sleep(1)
                    ep_chassis.move(x=x_dist, y=0, z=0, xy_speed=0.7).wait_for_completed()
                    done = True

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)

def wait():
    time.sleep(1.0)

# release and back away
def release():
    # open gripper
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    x_val = -5
    ep_chassis.move(x=x_val, y=0, z=0, xy_speed=0.7).wait_for_completed()

def returnToStartLeft():
    ep_chassis.move(x=0, y=-0.2, z=0, xy_speed=0.5).wait_for_completed() # go left 20 cm
    ep_chassis.move(x=-0.5, y=0, z=0, xy_speed=0.5).wait_for_completed() # go back 50 cm
    ep_chassis.move(x=0, y=0.3, z=0, xy_speed=0.5).wait_for_completed() # move right 30 cm
    ep_chassis.move(x=0.3, y=0, z=0, xy_speed=0.5).wait_for_completed() # move forward 30cm to enter lego pile

def returnToStartRight():
    ep_chassis.move(x=0, y=0.2, z=0, xy_speed=0.5).wait_for_completed() # go right 20 cm
    ep_chassis.move(x=-0.5, y=0, z=0, xy_speed=0.5).wait_for_completed() # go back 50 cm
    ep_chassis.move(x=0, y=-0.3, z=0, xy_speed=0.5).wait_for_completed() # move left 30 cm
    ep_chassis.move(x=0.3, y=0, z=0, xy_speed=0.5).wait_for_completed() # move forward 30cm to enter lego pile

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    model = YOLO('/Users/sachin/Documents/Coursework/CMSC477/CMSC477/big_model.pt')
    
    START = "LEFT"

    while(True):
        if START == "LEFT":
            resetArmPos()
            ep_chassis.move(x=0, y=0.6, z=0, xy_speed=0.5).wait_for_completed() # move right 60cm
            findLego(model)
            driveToLego(model)
            grabLego()
            ep_chassis.move(x=-0.3, y=0, z=0, xy_speed=0.5).wait_for_completed() # move backwards 30cm to escape u-shape
            ep_chassis.move(x=0, y=-0.3, z=0, xy_speed=0.5).wait_for_completed() # move left 30 cm
            ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.5).wait_for_completed() # go forwards 50 cm
            ep_chassis.move(x=0, y=0.2, z=0, xy_speed=0.5).wait_for_completed() # go right 20 cm
            findLine()
            driveToLine()
            wait()
            release()
            returnToStartLeft()
        else:
            resetArmPos()
            ep_chassis.move(x=0, y=-0.6, z=0, xy_speed=0.5).wait_for_completed() # move left 60cm
            findLego(model)
            driveToLego(model)
            grabLego()
            ep_chassis.move(x=-0.3, y=0, z=0, xy_speed=0.5).wait_for_completed() # move backwards 30cm to escape u-shape
            ep_chassis.move(x=0, y=0.3, z=0, xy_speed=0.5).wait_for_completed() # move right 30 cm
            ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.5).wait_for_completed() # go forwards 50 cm
            ep_chassis.move(x=0, y=-0.2, z=0, xy_speed=0.5).wait_for_completed() # go left 20 cm
            findLine()
            driveToLine()
            wait()
            release()
            returnToStartRight()