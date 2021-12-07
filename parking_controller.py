"""parking_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera
from vehicle import Car, Driver
import random
import numpy as np
import cv2

# create the Robot instance.
robot = Driver()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)


front_camera = robot.getDevice("front_camera")

front_camera.enable(20)
#rear_camera.enable(30)
lidar = robot.getDevice("Sick LMS 291")
lidar.enable(timestep)
lidar.enablePointCloud()
c= 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step() != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # Read Lidar           
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_left     = lidar_sensor_readings[0:60]
    lidar_sensor_center   = lidar_sensor_readings[60:120]
    lidar_sensor_right    = lidar_sensor_readings[120:180]
    # Getting rid of infinity outliers
    if lidar_sensor_left != 0:
        ll = np.array(lidar_sensor_left)
        lv = ll[ll != np.Inf]
    if lidar_sensor_center != 0:
        lc = np.array(lidar_sensor_center)
        cv = lc[lc != np.Inf]
    
    if lidar_sensor_right != []:
        lr = np.array(lidar_sensor_right)
        rv = lr[lr != np.Inf]
        
    # Process sensor data here.
    # print(robot.getControlMode())
    # print(robot.getThrottle())
    # robot.setGear(1)
    # print("Left Value: ", lv)
    robot.setGear(1)
        # brake       = 0.0    
        # steer_angle = 0
        # speed       = 35
    # elif lv < 3.7:
    brake       = 0
    steer_angle = 0
    speed       = 20   
    gear        = 0
    try:   
        # if np.min(rv) < 3.7:
        if rv.size ==0:
            c += 1
            if c < 10:
                brake       = 1.
                steer_angle = 0
                speed       = 15
                gear        = 0
                print("**********no parking")
                
            if c > 10:
               brake       = 1.
               steer_angle = -31
               speed       = 20
               gear        = -1
            if c > 150:
               brake       = 1.
               steer_angle = 31
               speed       = 20
               gear        = 0
            if c > 200:
               brake       = 1
               steer_angle = 0
               speed       = 0
               gear        = 0
            # if np.min(cv) > 0:
               # brake       = 0
               # steer_angle = -20
               # speed       = 10
               
               # c=0
            # elif lv.size != 0:
                # brake       = 1.
                # steer_angle = 31
                # speed       = 15
    except:
        print("**********except")

    robot.setBrakeIntensity(brake)
    robot.setSteeringAngle(steer_angle)
    robot.setCruisingSpeed(speed)
    robot.setGear(gear)
    
    #check for rv reading for couple of
    
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
