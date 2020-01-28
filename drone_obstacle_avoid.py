# Here is a video of the drone in action: https://www.youtube.com/watch?v=IMz2qh4fTZ4

group_number = 12

import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

# CrazyFlie imports:
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander


## Some helper functions:
## -----------------------------------------------------------------------------------------

# Determine initial position:
def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')
    
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')
    
    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10
    
    threshold = 0.001
    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)
            
            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)
            
            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))
                
            if (max_x - min_x) < threshold and (
                        max_y - min_y) < threshold and (
                        max_z - min_z) < threshold:
                  break

#display image
def displayImage(image):
    plt.imshow(image)
    plt.show()

# Ascend and hover:
def set_PID_controller(cf):
    # Set the PID Controller:
    print('Initializing PID Controller')
    cf.param.set_value('stabilizer.controller', '1')
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)
    return

# Ascend and hover:
def ascend_and_hover(cf):
    print('Ascending to hover:')
    # Ascend:
    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
        time.sleep(0.1)
    # Hover at 0.5 meters:
    for _ in range(30):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.5)
        time.sleep(0.1)
    return

# Follow the setpoint sequence trajectory:
def run_position(cf, position):

    print('Setting position {}'.format(position))
    for i in range(10):
        cf.commander.send_position_setpoint(position[0],
                                            position[1],
                                            position[2],
                                            position[3])
        time.sleep(0.1)

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
    if largest_contour_index > -1 and largest_area > 15000:
        print(largest_area)
        red = cv2.drawContours(frame, contours[largest_contour_index], -1, (0, 255, 0), 3)
        cv2.imshow('red', red)
        print(largest_area)
        x,y,w,h = cv2.boundingRect(contours[largest_contour_index])
        if x > (720/2 - 40):
            #print('move left')
            return 'L'
        else:
            print('move right')
            return 'R'
    #print(str(x))
    return 'F'

# Follow the setpoint sequence trajectory:
def adjust_position(cf, current_y):

    print('Adjusting position')

    steps_per_meter = int(10)
    for i in range(steps_per_meter):
        current_y = current_y - 1.0/float(steps_per_meter)
        position = [0, current_y, 0.5, 0.0]
        position = [x,y,.5,0]
        print('Setting position {}'.format(position))
        for i in range(10):
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
    return current_y
    
def hover(cf):
    for _ in range(30):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.5)
        time.sleep(0.1)

def move_forward(cf, x, y):
    new_x = x + .05
    new_y = y + 0
    position = [new_x, new_y]
    for i in range(3):
        cf.commander.send_position_setpoint(position[0],
                                            (position[1]),
                                            0.5,
                                            0.0)
        time.sleep(0.1)
    #cf.commander.send_stop_setpoint()
    #time.sleep(0.1)
    #hover(cf)
    print('move forward')
    print(f'Setting position {(position[0], (position[1]))}')
    return new_x, new_y

def move_left(cf, x, y):
    new_x = x + 0
    new_y = y - .05
    position = [new_x, new_y]
    for i in range(3):
        cf.commander.send_position_setpoint(position[0],
                                            (position[1]),
                                            0.5,
                                            0.0)
        time.sleep(0.1)

    # cf.commander.send_stop_setpoint()
    #time.sleep(0.1)
    # hover(cf)
    print('move left')
    print(f'Setting position {(position[0], (position[1]))}')
    return new_x, new_y

def move_right(cf, x, y):
    new_x = x + 0
    new_y = y + .05
    position = [new_x, new_y]
    for i in range(3):
        cf.commander.send_position_setpoint(position[0],
                                            (position[1]),
                                            0.5,
                                            0.0)
        time.sleep(0.1)
    #cf.commander.send_stop_setpoint()
    #time.sleep(0.1)
    #hover(cf)
    print('move right')
    print(f'Setting position {(position[0], (position[1]))}')
    return new_x, new_y

# Hover, descend, and stop all motion:
def hover_and_descend(cf):
    print('Descending:')
    # Hover at 0.5 meters:
    for _ in range(30):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.5)
        time.sleep(0.1)
    # Descend:
    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
        time.sleep(0.1)
    # Stop all motion:
    for i in range(10):
        cf.commander.send_stop_setpoint()
        time.sleep(0.1)
    return

## -----------------------------------------------------------------------------------------

## The following code is the main logic that is executed when this script is run

## -----------------------------------------------------------------------------------------

# Set the URI the Crazyflie will connect to
uri = f'radio://0/{group_number}/2M'

# Initialize all the CrazyFlie drivers:
cflib.crtp.init_drivers(enable_debug_driver=False)

# Scan for Crazyflies in range of the antenna:
print('Scanning interfaces for Crazyflies...')
available = cflib.crtp.scan_interfaces()
# List local CrazyFlie devices:
print('Crazyflies found:')
for i in available:
    print(i[0])

# Check that CrazyFlie devices are available:
if len(available) == 0:
    print('No Crazyflies found, cannot run example')
else:
    ## Ascent to hover; run the sequence; then descend from hover:
    # Use the CrazyFlie corresponding to team number:
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        right = True
        x = 0.0
        y = 0.0
        # Initialize and ascend:
        t = time.time()
        elapsed = time.time() - t
        ascended_bool = 0
            
        cap = cv2.VideoCapture(0)
        with MotionCommander(scf, default_height = .1) as mc:
            mc.land()
            while(cap.isOpened()):
                
                ret, frame = cap.read()
                
                elapsed = time.time() - t
                if(elapsed > 8.0):
                    
                    print('Capturing.....')
                    
                    if ret==True:
                        cv2.imshow('frame',frame)
                        # displayImage(frame)
                        pos = x, y
                        print('Curent position: ', pos)
                        if(ascended_bool==0):
                            mc.take_off(.5)
                            ascended_bool = 1
                            time.sleep(1)
                        
                        else:
                            dir = check_contours(frame)
                            if dir == 'R':
                                print('move right')
                                mc.right(.10)
                                y += .10
                            elif dir == 'L':
                                print('move left')
                                mc.left(.10)
                                y -= .10
                            else:
                                print('move forward')
                                mc.forward(.10)
                                x += .10
                            if x > 3.6:
                                break

            
            cap.release()

            # Descend and stop all motion:
            mc.land()

print('Done!')
