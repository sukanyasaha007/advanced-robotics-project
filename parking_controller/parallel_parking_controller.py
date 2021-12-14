"""parallel parking controller."""

from controller import Robot, Camera
from vehicle import Car, Driver
import random
import numpy as np
import cv2

# create the Robot instance.
robot = Driver()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# initializing LiDAR and GPS sensors
lidar = robot.getDevice("Sick LMS 291")
lidar.enable(timestep)
lidar.enablePointCloud()

gps = robot.getDevice("gps")
gps.enable(timestep)

# setting initial system state
state = 1
brake = 0.0
steer_angle = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step() != -1:
    # read the sensors    
    lidar_sensor_readings = lidar.getRangeImage()
    #lidar_sensor_left     = lidar_sensor_readings[0:60]
    lidar_sensor_center   = lidar_sensor_readings[60:120]
    lidar_sensor_right    = lidar_sensor_readings[120:180]    
    lidar_center = np.array(lidar_sensor_readings[85:95]) 
    
    location = gps.getValues()
    
    if state == 1:
        speed = 40
        slot_detected = all(ele == np.inf for ele in lidar_sensor_right)
        if slot_detected:
            pos1 = location[2]
            state = 2
            
    elif state == 2:
        speed = 20
        pos2 = location[2]
        dist_travelled = pos2 - pos1
        if dist_travelled > 14.0:
            state = 3
            
    elif state == 3:
        speed = 0
        if gps.getSpeed() < 2:
            pos1 = location[2]            
            state = 4
            
    elif state == 4:
        speed = -10
        pos2 = location[2]
        dist_travelled = pos1 - pos2
        if dist_travelled > 0.1:
            pos1 = location[2]
            state = 5
            
    elif state == 5:
        steer_angle = 0.52
        speed = -10
        pos2 = location[2]
        dist_travelled = pos1 - pos2
        if dist_travelled > 4.6:
            speed = -5
            steer_angle = -0.52
            pos1 = location[2]
            state = 6
            
    elif state == 6:
        speed = -5
        steer_angle = -0.52        
        pos2 = location[2]
        dist_travelled = pos1 - pos2
        if dist_travelled > 4.5:        
            state = 7
            speed = 0
            steer_angle = 0
  
    # apply state values
    robot.setBrakeIntensity(brake)
    robot.setSteeringAngle(steer_angle)
    robot.setCruisingSpeed(speed)