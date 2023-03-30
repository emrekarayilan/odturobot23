# Created for 19. METU Robot Days
# 30.03.2023
# ---Authors---
# Eren Guler - erenguler.ee@gmail.com
# Aliberk Serin - aliberksrn.work@gmail.com
# Emre Taha Karayilan - emretahakarayilan@gmail.com

# --- Importing Required Libraries ---
import math
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import argparse

# --- UAV Connection ---

vehicle = connect("/dev/ttyACM0", wait_ready=True)
print("vehicle is connected")


# --- Mission Control Variables ---
alt = 1  # Mission altitude
resolution = 180 # Resolution of moving pattern
starting_heading = vehicle.heading
r = 0.15 # Radius of inner circle defined for the movement pattern
b = 0.50 # Radius of outer circle defined for the movement pattern

clockwise = -1 # Clockwise motion of the pattern
c_clockwise = 1 # Counterclockwise motion of the pattern

vel_time = 0.04 # Determines time interval between MAV Commands
vel_magnitude_constant = 1 # Scales the size of one pattern
x_const = 0.8 # Scales only ax of the pattern
y_const = 1 # Scales only ay of the pattern


def arm_and_takeoff(tgt_alt):
    """
    Arm the vehicle and takeoff to a TargetAltitude.
    """

    print("Basic Pre-arm Checks")
    # Don't try to arm the vehicle until autopilot is ready
    while not vehicle.is_armable:
        time.sleep(1)
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode('GUIDED')
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        vehicle.armed = True
        time.sleep(1)

    vehicle.simple_takeoff(tgt_alt) # Takeoff to a target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).

    while True:
        if vehicle.location.global_relative_frame.alt >= tgt_alt * 0.95:
            break
        time.sleep(1)
    print("takeoff ended")


def set_ned_velocity_heading(vel_x,vel_y,vel_z,heading,duration):
    """
    Sets vehicles velocity according to vel_x, vel_y and vel_z variables. Also sets heading according to heading variable.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Frame of the vehicle
        # Bitmask to indicate which fields should be ignored by the vehicle. Value of '1' indicates that those bits will be ignored.
        # bit0:PosX, bit1:PosY, bit2:PosZ, bit3:VelX, bit4:VelY, bit5:VelZ, bit6:AccX, bit7:AccY, bit8:AccZ, bit10: Force/Acceleration ,bit10:yaw, bit11:yaw rate
        455,  # Use velocity, heading rate, heading information of the message and ignore others.
        0, 0, 0, # north,east and down position according to first EKF origin
        vel_y, vel_x, -vel_z, # north,east and down velocity according to first EKF Origin
        0, 0, 0, # north,east and down acceleration according to first EKF Origin
        heading, 0) # heading and heading_rate according to first EKF Origin

    for x in range(0, duration):
        vehicle.send_mavlink(msg) # send the message to vehicle with mavlink protocol
        time.sleep(vel_time) # wait until message is transmitted and executed


def rotate_vel_among_axis(vel_list,rot_degree):
    """
    Rotates given velocity list by rotation degree among cartesian coordinate system which coincides with EKF NED origin.
    """

    rot_rad = math.radians(-rot_degree) # Convert rotation degree to radians
    rotated_list = list()

    for i in range(0,len(vel_list)):
        rotated_list.append([(((vel_list[i][0] *math.cos(rot_rad)) - (vel_list[i][1]) *math.sin(rot_rad))), # Rotate x (East)
                             ((vel_list[i][0] * math.sin(rot_rad) + (vel_list[i][1]) * math.cos(rot_rad))), # Rotate y (North)
                             vel_list[i][2] - rot_rad]) # Rotate the heading (in radians; 0 is North)

    return rotated_list


def pattern_creator(r,b,resolution,clockwise,vel_const):
    """
    Crete lists of items that will lead vehicle to complete one mission pattern.
    """

    t = 0 # This is Theta in parametric equation of Prolate Cycloic formula. It could be found in : https://mathworld.wolfram.com/ProlateCycloid.html
    vel_list = list()
    pose_list = list()
    center_coordinate = [-(b - r), (2 * b)]  # Coordinate of center point of the circle which will form with completion of one pattern.

    for i in range(0, resolution):
        t = 2 * math.radians(i) # Converting from degree to radians

        """
        Calculating required velocity to achieve proper position of next iteration of pattern.
        As you might see, this function is the derivative of the position function. 
        """

        y_vel = (r - b * math.cos(t)) * vel_const * y_const
        x_vel = (clockwise * b * math.sin(t)) * vel_const * x_const

        # Calculating required position to achieve proper position of pattern.
        y_pose = r * t - b * math.sin(t)
        x_pose = clockwise * ((r - b * math.cos(t)) + (b - r))

        """
        In ideal situation, in next iteration the vehicle will be at current_coordinate coordintes. By using this coordinate
        required heading to look at center of the circle could be found. 
        """

        current_coordinate = [x_pose,y_pose]
        the_heading = math.radians(find_center_heading(current_coordinate,center_coordinate)) # This returns heading in radians

        vel_list.append([x_vel, y_vel,the_heading])
        pose_list.append([x_pose, y_pose])

    return vel_list, pose_list


def find_center_heading(current_coordinate,center_coordinate):
    """
    This function finds required heding to look at the center of circle.
    """

    angle = math.degrees(math.atan((center_coordinate[0]-current_coordinate[0])/(center_coordinate[1]-current_coordinate[1])))
    if (center_coordinate[1]-current_coordinate[1]) < 0:
        angle = 180 + angle

    return angle


def draw_pattern_vel_ned(vel_list,rotation,count):
    """
    Get the velocity list and iterate it with how many times it is required to be repeated.
    """

    pattern_vel = rotate_vel_among_axis(vel_list,rotation) # Rotates items of list with required rotation in cartesian.

    for j in range(0,count):
        for i in range(0,resolution):
            set_ned_velocity_heading(pattern_vel[i][0], pattern_vel[i][1], 0, pattern_vel[i][2], 1)
        print(f"pattern {j} ended")


def do_mission_vel_ned(pattern,start_heading):
    draw_pattern_vel_ned(pattern, start_heading + 0, 5)
    print(f"First line to forward completed, curve starterd")
    draw_pattern_vel_ned(pattern, start_heading + 45, 1)
    print(f"Curve of 1m radius circle done, going to straight line to right")
    draw_pattern_vel_ned(pattern, start_heading + 90, 3)
    print(f"Straight line to right done, going straight line to down")
    draw_pattern_vel_ned(pattern, start_heading + 180, 6)
    print(f"Straight line to down done, first curve of second circle started")
    draw_pattern_vel_ned(pattern, start_heading + 135, 2)
    print(f"First curve of 2m radius circle done, second curve of the circle started")
    draw_pattern_vel_ned(pattern, start_heading + 45, 2)
    print(f"Second curve of 2m radius circle done, last up line started")
    draw_pattern_vel_ned(pattern, start_heading + 0, 6)
    print(f"Last straight line to front done, mission ended.landing")


if __name__ == '__main__':

    arm_and_takeoff(alt)

    pattern_vel_list, pattern_pose_list = pattern_creator(r,b,resolution,clockwise,vel_magnitude_constant)

    do_mission_vel_ned(pattern_vel_list,starting_heading)

    print("Landing Mode")
    vehicle.mode = 'LAND'