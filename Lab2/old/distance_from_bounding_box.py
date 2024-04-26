import numpy as np

distanceSignal = []

def distance_from_box_size(boxWidth, boxHeight):

    #param
    a = 80.8847
    b = -214.7402
    c = 186.6250
    expected_aspect_ratio = 7 #h/w

    actual_aspect_ratio = boxHeight / boxWidth
    correction = expected_aspect_ratio/actual_aspect_ratio
    actualHeight = boxHeight * correction
    
    h_distance = a * actualHeight^2 + b * actualHeight + c 
    distanceSignal.append(h_distance)

    #rolling median filter
    if(len(distanceSignal) >= 3):
        h_distance =  np.median(distanceSignal[-3:])

    return h_distance
