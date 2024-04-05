import numpy as np

def distance_from_box_size(boxWidth, boxHeight):

    expected_aspect_ratio = 7 #h/w
    actual_aspect_ratio = boxHeight / boxWidth
    correction = expected_aspect_ratio/actual_aspect_ratio
    actualHeight = boxHeight * correction

    gain = 1
    offset = 1


    h_distance = gain * actualHeight + offset

    return h_distance