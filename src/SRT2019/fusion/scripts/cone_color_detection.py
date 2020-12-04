#!/usr/bin/env python
import numpy as np
import glob   
from PIL import Image
import matplotlib.pyplot as plt
import pylab
import cv2
import string


def color_detection(cone_img):
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(cone_img, cv2.COLOR_BGR2HSV)
    #hsv = cone_img

    # define range of blue color in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue = len(mask[mask > 0])
    print ("blue size")
    print blue
    
    
    # define range of red color in HSV
    lower_red = np.array([0,43,36])
    upper_red = np.array([20,255,255])
    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)
    red = len(mask[mask > 0])

    lower_red = np.array([146,43,36])
    upper_red = np.array([180,255,255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    red = red + len(mask[mask > 0])

    print ("red size")
    print red

    # define range of yellow color in HSV
    lower_yellow = np.array([26,43,46])
    upper_yellow = np.array([34,255,255])
    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow = len(mask[mask > 0])

    print ("yellow size")
    print yellow

    color = 'unknown'
    threshold = len(cone_img) / 10

    if blue > yellow and blue > red and blue > threshold:
        color = 'b'
    elif red > yellow and red > blue and red > threshold:
        color = 'r'
    elif yellow > blue and yellow > red and yellow > threshold:
        color = 'y'

    

    return color

