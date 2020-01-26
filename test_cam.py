import numpy as np
import cv2
import time

def findGreatesContour(contours):
    largest_area = 0
    largest_contour_index = -1
    i = 0
    total_contours = len(contours)

    while i < total_contours:
        area = cv2.contourArea(contours[i])
        if area > largest_area:
            largest_area = area
            largest_contour_index = i
        i += 1

    #print(largest_area)

    return largest_area, largest_contour_index

def check_contours(frame):

    print('Checking image:')

    # These define the upper and lower HSV for the red obstacles
    # May change on different drones.
    # Do the contour detection on the input frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, (0,50,20), (5,255,255))
    mask2 = cv2.inRange(hsv, (175,50,20), (180,255,255))

    ## Merge the mask and crop the red regions
    mask = cv2.bitwise_or(mask1, mask2)
    
    ## Display
    cv2.imshow("mask", mask)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    largest_area, largest_contour_index = findGreatesContour(contours)
    if largest_contour_index > -1:
        red = cv2.drawContours(frame, contours[largest_contour_index], -1, (0, 255, 0), 3)
        cv2.imshow('red', red)
        print(largest_area)
        x,y,w,h = cv2.boundingRect(contours[largest_contour_index])
        if x > (720/2):
            print('move left')
        else:
            print('move right')
        print(str(x))

    if largest_area > 100:
        return True
    else:
        return False

def apply_brightness_contrast(input_img, brightness = 255, contrast = 127):
    brightness = map(brightness, 0, 510, -255, 255)
    contrast = map(contrast, 0, 254, -127, 127)

    if brightness != 0:
        if brightness > 0:
            shadow = brightness
            highlight = 255
        else:
            shadow = 0
            highlight = 255 + brightness
        alpha_b = (highlight - shadow)/255
        gamma_b = shadow

        buf = cv2.addWeighted(input_img, alpha_b, input_img, 0, gamma_b)
    else:
        buf = input_img.copy()

    if contrast != 0:
        f = float(131 * (contrast + 127)) / (127 * (131 - contrast))
        alpha_c = f
        gamma_c = 127*(1-f)

        buf = cv2.addWeighted(buf, alpha_c, buf, 0, gamma_c)

    #cv2.putText(buf,'B:{},C:{}'.format(brightness,contrast),(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    return buf

def map(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

# This may open your webcam instead of the CrazyFlie camera! If so, try
# a different small, positive integer, e.g. 1, 2, 3.
cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    img = apply_brightness_contrast(frame, 200, 150)
    # img = apply_brightness_contrast(frame)
    # Display the resulting frame
    #cv2.imshow('img', img)
    print(check_contours(frame))
    time.sleep(3)
    # Hit q to quit.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
