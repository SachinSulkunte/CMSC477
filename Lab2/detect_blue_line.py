import cv2
import numpy as np

# Function to detect blue line orientation and distance
def detect_blue_line(frame):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of blue color in HSV
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        #  largest contour should be blue line
        blue_contour = max(contours, key=cv2.contourArea)
        
        # fit line
        [vx, vy, x, y] = cv2.fitLine(blue_contour, cv2.DIST_L2, 0, 0.01, 0.01)
        
        # calc angle from line direction
        angle_deg = np.arctan2(vy, vx) * 180 / np.pi

        # check if blue line is in the bottom 10% of pixels of the image
        bottom_percentile = frame.shape[0] * 0.9
        bottom_of_line = max(blue_contour[:, 0, 1])
        is_close = bottom_of_line >= bottom_percentile

        return angle_deg, is_close

    return None, None

# # Capture video from camera
# cap = cv2.VideoCapture(0)

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break

#     # Detect blue line
#     angle, is_close = detect_blue_line(frame)

#     if angle is not None:
#         # Draw line indicating orientation
#         cv2.line(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), (int(frame.shape[1]/2) + 100, int(frame.shape[0]/2) + int(100*np.tan(angle * np.pi / 180))), (0, 255, 0), 2)
#         print("Angle: " + str(angle))
        
#         if is_close:
#             cv2.putText(frame, "Blue line is close", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#     cv2.imshow('Frame', frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
cv2.destroyAllWindows()
