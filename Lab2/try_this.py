import cv2
import numpy as np
from matplotlib import pyplot as plt

frame = cv2.imread("a.jpeg")
img = frame
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
lower_blue= np.array([78,158,124])
upper_blue = np.array([138,255,255])

mask = cv2.inRange(img,lower_blue,upper_blue)

img,cnts,hie = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
cv2.drawContours(frame,cnts,-1,(0,255,0),3)

cv2.imshow("Frame",frame)
cv2.waitKey(0)
