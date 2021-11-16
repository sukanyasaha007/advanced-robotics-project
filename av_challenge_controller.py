"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera
from vehicle import Car, Driver
import random
import numpy as np
import cv2
# create the Robot instance.
robot = Driver()
front_camera = robot.getDevice("front_camera")
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

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step() != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # Read camera
    image = front_camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((front_camera.getHeight(), front_camera.getWidth(), 4))
    frame = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    #print(frame[0][60])
    cv2.imshow("frame", frame)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # HSV
    lower_gray = np.array([100, 10, 30])
    upper_gray = np.array([120, 25, 80])
    
    # Threshold the HSV image to get only gray colors
    mask = cv2.inRange(hsv, lower_gray, upper_gray)
    
    cv2.imshow("mask", mask)
    
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)
    #print(res[front_camera.getHeight()-1][0])
    #print(res[front_camera.getHeight()-1][127])
    #print("**************")
    
    cv2.imshow('res', res)
    
    cv2.waitKey(1) # Render imshows on screen
    
    # Read Lidar           
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_left     = lidar_sensor_readings[0:60]
    lidar_sensor_center   = lidar_sensor_readings[60:120]
    lidar_sensor_right    = lidar_sensor_readings[120:180]
    # Getting rid of infinity outliers
    ll = np.array(lidar_sensor_left)
    lv = np.min(ll[ll != np.Inf])
    
    lc = np.array(lidar_sensor_center)
    cv = np.min(lc[lc != np.Inf])
    
    lr = np.array(lidar_sensor_right)
    rv = np.min(lr[lr != np.Inf])
        
    # Process sensor data here.
    # print(robot.getControlMode())
    # print(robot.getThrottle())
    robot.setGear(1)
    #print("Left Value: ",lv)
    #print("Right Value: ",rv)
    #print("**********")
    
    if lv > 10 and rv > 10:
        if res[front_camera.getHeight()-6][0][0] == 0 and res[front_camera.getHeight()-6][0][1] == 0 and res[front_camera.getHeight()-6][0][2] == 0:
            brake       = 1.
            steer_angle = -31
            speed       = 20
        elif res[front_camera.getHeight()-6][127][0] == 0 and res[front_camera.getHeight()-6][127][1] == 0 and res[front_camera.getHeight()-6][127][2] == 0:
            brake       = 1.
            steer_angle = 31
            speed       = 20
        else:
            brake       = 0.0    
            steer_angle = 0
            speed       = 40
    else:
        if lv > 3.7 and rv > 3.7:
            brake       = 0.0    
            steer_angle = 0
            speed       = 40
        elif lv < 3.7:
            brake       = 1.
            steer_angle = -31
            speed       = 20      
        elif rv < 3.7:
            brake       = 1.
            steer_angle = 31
            speed       = 20
    
    robot.setBrakeIntensity(brake)
    robot.setSteeringAngle(steer_angle)
    robot.setCruisingSpeed(speed)

