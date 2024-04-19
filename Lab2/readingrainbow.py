import numpy as np
import matplotlib
import imutils
import cv2
import matplotlib.pyplot as plt

aa = 300; cc = 50

def grab_frame(camera):
    ret, frame = camera.read()
    return frame

def snip_image(img):

    snip = img[(img.shape[0] - aa):(img.shape[0] - cc), (0):(img.shape[1]-100)]
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
    cv2.imshow("hsv",hsv)
    cv2.imshow("hsv blue mask", blue_mask)
    thresholded = cv2.bitwise_and(img,img, mask=blue_mask)

    return thresholded

def blur_img(img):
    return cv2.GaussianBlur(img,(15,15),0)

def edge_img(img):
    return cv2.Canny(img, 30, 50)

def line_image(img):
    return cv2.HoughLines(img, 1, np.pi/180, 10)

def plot_Hough_Lines(img, rho, theta, blue, green, red):
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    x2 = int(x0 - 1000*(-b))
    y1 = int(y0 + 1000*a)
    y2 = int(y0 - 1000*a)
    cv2.line(img, (x1,y1), (x2,y2), (blue*255, green*255, red*255), 10)


def main():

    # camera = cv2.VideoCapture('lanedetectiontestvideo.mp4')

    # image = grab_frame(camera)

    image = cv2.imread("robot_bllllue.jpg")

    cv2.imshow("Caption",image)

    snip = snip_image(image)
    cv2.imshow("Snip",snip)

    thresholded= thres_image(snip)
    cv2.imshow("Thresholded Snip",thresholded)

    blurred = blur_img(thresholded)
    cv2.imshow("Blurred Image", blurred)

    edged = edge_img(blurred)
    cv2.imshow("Edged", edged)

    lines = line_image(edged)
    print(lines)



    print("---")
    cv2.waitKey(0)

if __name__ == '__main__':
    main()