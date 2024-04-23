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
    #cv2.imshow("hsv",hsv)
    #cv2.imshow("hsv blue mask", blue_mask)
    thresholded = cv2.bitwise_and(img,img, mask=blue_mask)

    return thresholded

def blur_img(img):
    return cv2.GaussianBlur(img,(15,15),0)

def edge_img(img):
    return cv2.Canny(img, 30, 50)

def line_image(img):
    return cv2.HoughLines(img, 1, np.pi/180, 20)

def plot_Hough_Lines(img, rho, theta):
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    x2 = int(x0 - 1000*(-b))
    y1 = int(y0 + 1000*a)
    y2 = int(y0 - 1000*a)
    cv2.line(img, (x1,y1), (x2,y2), (0, 0, 255), 5)

    a1 = np.cos(theta-theta)
    b1 = np.sin(theta-theta)
    x01 = a*rho
    y01 = b*rho
    x11 = int(x01 + 1000*(-b1))
    x21 = int(x01 - 1000*(-b1))
    y11 = int(y01 + 1000*a1)
    y21 = int(y01 - 1000*a1)

    cv2.line(img, (x11+272,y11), (x21+272,y21), (0, 255, 0), 5)
    return img

def getAngle():
        
    image = cv2.imread("robot_blorange.jpg")

    #cv2.imshow("Caption",image)

    snip = snip_image(image)
    #cv2.imshow("Snip",snip)

    thresholded= thres_image(snip)
    #cv2.imshow("Thresholded Snip",thresholded)

    blurred = blur_img(thresholded)
    #cv2.imshow("Blurred Image", blurred)

    edged = edge_img(blurred)
    #cv2.imshow("Edged", edged)

    lines = line_image(edged)
    print(lines)

    if lines != None:
    
        rho = []
        theta = []
        for i in range(0, len(lines)):
            for r, o in lines[i]:
                rho.append(r)
                theta.append(o)
        print(rho)
        print(theta)
        print("---")
        rho_avg = np.mean(rho)
        theta_avg = np.mean(theta)
        angle = (180/np.pi)*theta_avg

        final = plot_Hough_Lines(snip,rho_avg,theta_avg)
        cv2.imshow("Final",final)

        print(angle)




        print("---")
        cv2.waitKey(0)

        return angle



# def main():

#     # camera = cv2.VideoCapture('lanedetectiontestvideo.mp4')

#     # image = grab_frame(camera)

#     image = cv2.imread("robot_blorange.jpg")

#     #cv2.imshow("Caption",image)

#     snip = snip_image(image)
#     #cv2.imshow("Snip",snip)

#     thresholded= thres_image(snip)
#     #cv2.imshow("Thresholded Snip",thresholded)

#     blurred = blur_img(thresholded)
#     #cv2.imshow("Blurred Image", blurred)

#     edged = edge_img(blurred)
#     #cv2.imshow("Edged", edged)

#     lines = line_image(edged)
#     print(lines)


#     rho = []
#     theta = []
#     for i in range(0, len(lines)):
#         for r, o in lines[i]:
#             rho.append(r)
#             theta.append(o)
#     print(rho)
#     print(theta)
#     print("---")
#     rho_avg = np.mean(rho)
#     theta_avg = np.mean(theta)
#     angle = (180/np.pi)*theta_avg

#     final = plot_Hough_Lines(snip,rho_avg,theta_avg)
#     cv2.imshow("Final",final)

#     print(angle)




#     print("---")
#     cv2.waitKey(0)

# if __name__ == '__main__':
#     main()