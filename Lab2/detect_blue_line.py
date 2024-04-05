import cv2
import numpy as np

image = cv2.imread('Lab 2/testing/robot_bllllue.jpg')

# Convert BGR to HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# blue HSV range
lower_blue = np.array([100, 50, 50])
upper_blue = np.array([130, 255, 255])

# threshold to only blue 
mask = cv2.inRange(hsv, lower_blue, upper_blue)

# get contours
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# filter the blue line based on size, shape, etc.
for contour in contours:
    area = cv2.contourArea(contour)
    if area > 100:  # adjust maybe
        # fit line to contour
        vx, vy, x, y = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        slope = vy / vx
     
        lefty = int((-x * vy / vx) + y)
        righty = int(((image.shape[1] - x) * vy / vx) + y)
        cv2.line(image, (image.shape[1] - 1, righty), (0, lefty), (0, 255, 0), 2)

# display image
cv2.imshow(' Blue Line', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
