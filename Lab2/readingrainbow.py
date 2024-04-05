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
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("Grayscale", gray)
    # thresh = 60

    # thresholded = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    whiteLower = np.array([148,100,43])
    whiteUpper = np.array([148,100,85])



    white_mask = cv2.inRange(hsv, whiteLower, whiteUpper)
    cv2.imshow("hsv",hsv)
    cv2.imshow("hsv mask", white_mask)
    thresholded = cv2.bitwise_and(img,img, mask=white_mask)
    return hsv

def blur_img(img):
    return cv2.GaussianBlur(img,(15,15),0)

def edge_img(img):
    return cv2.Canny(img, 30, 50)

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

    cv2.waitKey(0)


if __name__ == '__main__':
    main()