"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera
from vehicle import Car, Driver
import random
import numpy as np
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

