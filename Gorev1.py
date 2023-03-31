# Created for 19. METU Robot Days
# 30.03.2023
# ---Authors---
# Eren Guler - erenguler.ee@gmail.com
# Aliberk Serin - aliberksrn.work@gmail.com
# Emre Taha Karayilan - emretahakarayilan@gmail.com


import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import cv2
import numpy as np
import math

#--- UAV connection ---
vehicle = connect("/dev/ttyACM0", wait_ready=True)
vehicle.mode = VehicleMode("GUIDED")

# --- Cam Init ---
cap = cv2.VideoCapture(0)
frameWidth = 640
frameHeight = 480
deadZone=30

#------- Mission Control Variables------

red_counter=0
yellow_counter = 0
end_flag=False
yellow_detect_flag = False

#---------------------------------------

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and takeoff to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    time.sleep(2)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def send_body_velocity_rate(velocity_f, velocity_r, velocity_down, rate, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000010111000111,  # use only speeds and yaw_rate
        0, 0, 0,  # x, y, z positions (not used)
        velocity_f, velocity_r, velocity_down,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, rate)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.007)

def set_vel_vec(vx, vy):
    """
    Sets vehicles speed and yaw based on the camera feed.

    """
    if vx != 0 or vy != 0:
        max_yaw = 0.4 # Maximum yaw rate

        x_vector = 0.5 * (vx) / math.sqrt((vx) ** 2 + (vy) ** 2) # Sets x vector velocity magnitude
        y_vector = 0.5 * (vy) / math.sqrt((vx) ** 2 + (vy) ** 2) # Sets x vector velocity magnitude

        if y_vector == 0: y_vector = 0.00001 # Corrects ZeroDivisionError

        angle = math.atan(x_vector/y_vector) # Target yaw angle
        yaw_rate = vx * 0.005 * abs(math.sin(angle)) # Generating Yaw rate from target yaw angle

        if -10 < math.degrees(angle) < 10: # Deadzone for small angles
            yaw_rate = 0

        yaw_rate = min(max(yaw_rate, -max_yaw), max_yaw) # Constrates yaw rate

        # Sends generated velocity vectors and yaw rate to vehicle
        send_body_velocity_rate(0.8 * y_vector, 0.8 * x_vector, 0, yaw_rate, 1)

        print(f"y vector: {y_vector}\n"
              f"X_vector: {x_vector}\n"
              f"Rate: {yaw_rate}\n"
              "\n"
              )

    else:
        send_body_velocity_rate(0,0,0,0,1) # zero velocity vector


def display(img):
    """
    Draws lines for visualization to cam feed
    """
    global deadZone
    pt1 = ((int(frameWidth/2)-deadZone), (int(frameHeight/2)-deadZone)) # Deadzone area upper left corner
    pt2 = ((int(frameWidth/2)+deadZone), (int(frameHeight/2)+deadZone)) # Deadzone area bottom right corner

    cv2.line(img, (int(frameWidth / 2), 0), (int(frameWidth / 2), frameHeight), (110, 57, 4), 2)
    cv2.line(img, (0, int(frameHeight / 2)), (frameWidth, int(frameHeight / 2)), (110, 57, 4), 2)
    cv2.circle(img, (int(frameWidth / 2), int(frameHeight / 2)), 10, (110, 57, 4), 2)
    cv2.rectangle(img, pt1, pt2, (110, 57, 4), 2)


def red_detect(imageFrame):
    """
    Computes targets center point and sends to the set_vel_vec() function
    """
    global red_counter
    global end_flag
    global deadZone
    global yellow_detect_flag

    cv2.putText(img=imageFrame, text=f"Car Detect Mode", org=(10, 30),
                fontFace=1, fontScale=2, color=(110, 57, 4),
                thickness=2)

    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)#Converts cam feed to HSV space

    red_lower = np.array([155, 105, 0], np.uint8) # Lower boundary for red mask
    red_upper = np.array([179, 194, 230], np.uint8) # Upper boundary for red mask

    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) # Apply red mask to HSV image

    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE) # Find all contours in masked image

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour) # Computes contour area
        areaMin = 150 # Minimum area for contour
        if (area > areaMin): # Checks for minimum contour area

            x, y, w, h = cv2.boundingRect(contour) # Gets contours dimensions
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2) # Draw rectangle on contour

            cx = int(x + (w / 2)) # Contours center x point
            cy = int(y + (h / 2)) # Contours center y point

            vx = cx - (frameWidth / 2) # Contours center x point on cartesian coordinate system
            vy = (frameHeight / 2) - cy # Contours center y point on cartesian coordinate system

            cv2.putText(img=imageFrame,text=f"Vx:{vx} Vy:{vy}",org=(10,50),
                        fontFace=1,fontScale=1,color=(110, 57, 4),thickness=1)
            cv2.circle(imageFrame, (cx, cy), 5, (51, 255, 51), 1) # Draws circle on contours center
            set_vel_vec(vx,vy) # Sends computed vx,vy values to velocity controller function

            # Checks targets center point is in deadzone
            if (-deadZone <= vx <= deadZone) and (-deadZone <= vy <= deadZone):
                cv2.putText(img=imageFrame, text=f"Red_Counter:{red_counter}", org=(500, 30),
                            fontFace=1, fontScale=1, color=(110, 57, 4), thickness=1)

                set_vel_vec(0,0) # Stops vehicle if target is in deadzone

                red_counter+=1
                print(f"Counter:{red_counter}")
                if red_counter>=100:
                    print("Deadzone Acquired. Initiating Landing Process...")
                    time.sleep(1)
                    yellow_detect_flag=True
                    break


def yellow_detect(imageFrame):
    """
    Detects Landing Zone
    """
    global yellow_counter
    global end_flag
    global deadZone
    cv2.putText(img=imageFrame, text=f"Landing Mode", org=(10, 30),
                fontFace=1, fontScale=2, color=(110, 57, 4),
                thickness=2)

    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) #Converts cam feed to HSV space

    yellow_lower = np.array([0, 109, 71], np.uint8) # Lower boundary for yellow mask
    yellow_upper = np.array([36,255, 255], np.uint8)  # Lower boundary for yellow mask

    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper) # Apply yellow mask to HSV image

    contours, hierarchy = cv2.findContours(yellow_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE) # Find all contours in masked image

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour) # Computes contour area,

        areaMin = 1000  # Minimum area for contour

        if (area > areaMin): # Checks for minimum contour area
            x, y, w, h = cv2.boundingRect(contour) # Gets contours dimensions
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2) # Draw rectangle on contour

            cx = int(x + (w / 2)) # Contours center x point
            cy = int(y + (h / 2)) # Contours center y point

            vx = cx - (frameWidth / 2) # Contours center x point on cartesian coordinate system
            vy = (frameHeight / 2) - cy # Contours center y point on cartesian coordinate system

            cv2.circle(imageFrame, (cx, cy), 5, (51, 255, 51), 1) # Draws circle on contours center
            set_vel_vec(vx,vy)  # Sends computed vx,vy values to velocity controller function

            # Checks targets center point is in deadzone
            if (-deadZone<= vx <=deadZone) and (-deadZone <= vy <= deadZone):
                cv2.putText(img=imageFrame, text=f"Yellow_Counter:{yellow_counter}", org=(470, 30),
                            fontFace=1, fontScale=1, color=(110, 57, 4), thickness=1)
                set_vel_vec(0,0)  # Stops vehicle if target is in deadzone
                yellow_counter+=1
                if yellow_counter>50:
                    print("Landing Zone Confirmed. Landing")
                    vehicle.mode=VehicleMode("LAND")
                    end_flag=True
                    break



def main_func():
    global end_flag
    global yellow_detect_flag
    while not end_flag:

        _, img = cap.read() # Gets cam feed from camera

        if not yellow_detect_flag:
            red_detect(img)
        else:
            yellow_detect(img)

        display(img)
        cv2.imshow("img_copy", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("Mission Completed.")

#main
if __name__ == '__main__':
    arm_and_takeoff(3.5)
    time.sleep(2) # Wait for vehicle to stabilize
    main_func()