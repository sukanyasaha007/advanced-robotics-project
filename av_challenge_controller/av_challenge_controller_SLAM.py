"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera
from vehicle import Car, Driver
import random
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 20 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# create the Robot instance.
robot = Driver()
front_camera = robot.getDevice("front_camera")
display = robot.getDevice("display")
#rear_camera = robot.getCamera("rear_camera")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

front_camera.enable(20)
#rear_camera.enable(30)
lidar = robot.getDevice("Sick LMS 291")
lidar.enable(timestep)
lidar.enablePointCloud()

lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
#lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] 
lidar_offsets = lidar_offsets[0:len(lidar_offsets)] 

gps = robot.getDevice("gps")
gps.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)

map = np.zeros(shape=[360,360])

state = 1
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step() != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    # ************ Controller *****************
    
    # GPS & Compass
    # Ground truth pose
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[2]

    n   = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    
    # Read Lidar           
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_left     = lidar_sensor_readings[0:60]
    lidar_sensor_center   = lidar_sensor_readings[60:120]
    lidar_sensor_right    = lidar_sensor_readings[120:180]
    
    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue #rho = LIDAR_SENSOR_MAX_RANGE

        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        wx =  math.cos(rad)*rx - math.sin(rad)*ry + pose_x
        wy =  -(math.sin(rad)*rx + math.cos(rad)*ry) + pose_y
        
        if rho < LIDAR_SENSOR_MAX_RANGE:
             try:
                 map[60+int(wx*3)][230+int(wy*3)] += 0.005
                 if map[60+int(wx*3)][230+int(wy*3)] > 1:
                     map[60+int(wx*3)][230+int(wy*3)] = 1
                 g = int(map[60+int(wx*3)][230+int(wy*3)]*255)
                 display.setColor(int(g*256**2+g*256+g))
                 display.drawPixel(60+int(wx*3), 230+int(wy*3))
             except:
                 pass
    print(60 + int(pose_x*3), 230 + int(pose_y*3))             
    display.setColor(int(0xFF0000))
    #display.drawPixel(int(pose_y),int(pose_x))
    display.drawPixel(60 + int(pose_x*3), 230 + int(pose_y*3))
    
    # Getting rid of infinity outliers
    try:
        ll = np.array(lidar_sensor_left)
        lv = np.min(ll[ll != np.Inf])
        
        lc = np.array(lidar_sensor_center)
        cv = np.min(lc[lc != np.Inf])
        
        lr = np.array(lidar_sensor_right)
        rv = np.min(lr[lr != np.Inf])
    except:
        pass
    
    # Read camera
    image = front_camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((front_camera.getHeight(), front_camera.getWidth(), 4))
    frame = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)    
    cv2.imshow("frame", frame)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # HSV
    lower_gray = np.array([100, 10, 30])
    upper_gray = np.array([120, 25, 80])
    
    lower_shadow = np.array([0, 0, 0])
    upper_shadow = np.array([140, 100, 40])
    
    # Threshold the HSV image to get only gray colors
    mask_road   = cv2.inRange(hsv, lower_gray, upper_gray)
    mask_shadow = cv2.inRange(hsv, lower_shadow, upper_shadow)
    
    # Bitwise-AND mask and original image
    res_shadow = cv2.bitwise_and(frame, frame, mask=mask_shadow)    
    
    #Edge detection
    edges = cv2.Canny(frame, 50, 150)
    
    is_left_edge  = any(edges[50:63, 0])
    is_right_edge = any(edges[50:63, 127])
    
    edges_shadow = cv2.Canny(res_shadow, 50, 150)
    
    is_left_edge_shadow  = any(edges_shadow[50:63, 0:5].flatten())
    is_right_edge_shadow = any(edges_shadow[50:63, 122:127].flatten())
    
    cv2.imshow('Edge', edges)
    cv2.imshow('Edge Shadow', edges_shadow)
    
    cv2.waitKey(1) # Render imshows on screen
    
    # Process sensor data here.
    # print(robot.getControlMode())
    # print(robot.getThrottle())
    robot.setGear(1)    
                  
    if state == 1:    
        if lv > 3.7 and rv > 3.7:
            brake       = 0.0    
            steer_angle = 0
            speed       = 8
        elif lv < 3.7:
            brake       = 1.
            steer_angle = -31
            speed       = 4      
        elif rv < 3.7:
            brake       = 1.
            steer_angle = 31
            speed       = 4
        
        if lv > 10 and rv > 10:
            state = 2
    elif state == 2:
        if (is_left_edge and is_right_edge) or (not is_left_edge and not is_right_edge):
            brake       = 0.0    
            steer_angle = 0
            speed       = 8
        elif not is_right_edge:
            brake       = 1.
            steer_angle = -31
            speed       = 4      
        elif not is_left_edge:
            brake       = 1.
            steer_angle = 31
            speed       = 4
        
        if res_shadow[front_camera.getHeight()-1][0][0] != 0 and res_shadow[front_camera.getHeight()-1][0][1] != 0 and res_shadow[front_camera.getHeight()-1][0][2] != 0 and res_shadow[front_camera.getHeight()-1][127][0] != 0 and res_shadow[front_camera.getHeight()-1][127][1] != 0 and res_shadow[front_camera.getHeight()-1][127][2] != 0:
            state = 3
    elif state == 3:
        if (is_left_edge_shadow and is_right_edge_shadow) or (not is_left_edge_shadow and not is_right_edge_shadow):
            brake       = 0.0    
            steer_angle = 0
            speed       = 8
        elif (is_left_edge_shadow and not is_right_edge_shadow):
            brake       = 1.
            steer_angle = -31
            speed       = 4    
        elif (not is_left_edge_shadow and is_right_edge_shadow):
            brake       = 1.
            steer_angle = 31
            speed       = 4
    
        if lv < 4.8 and cv < 6.1 and rv < 17.6:            
            state = 1
            brake = 1.
            speed = 0            
        
    robot.setBrakeIntensity(brake)
    robot.setSteeringAngle(steer_angle)
    robot.setCruisingSpeed(speed)
