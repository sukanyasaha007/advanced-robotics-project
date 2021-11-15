"""av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera
from vehicle import Car, Driver
import random

# camera realated packages
import numpy as np
import cv2 
from skimage.metrics import structural_similarity as compare_ssim
import imutils


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


front_camera.enable(100)

#rear_camera.enable(30)
lidar = robot.getDevice("Sick LMS 291")
lidar.enable(timestep)
lidar.enablePointCloud()

# cv2.startWindowThread()
# cv2.namedWindow("preview")

# Main loop:
# - perform simulation steps until Webots is stopping the controller

# cameraData = front_camera.getImage();
# print(cameraData)
# imageData = np.frombuffer(cameraData, np.uint8).reshape((front_camera.getHeight(), front_camera.getWidth(), 4))
arr3d= np.zeros((64, 128, 4))
new_image1 = arr3d.astype(np.uint8)
grayA = cv2.cvtColor(new_image1, cv2.COLOR_BGR2GRAY)

counter = 0

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
    
    # reading from camera
    #image 1
    # cameraData = front_camera.getImage();
    # imageData = np.frombuffer(cameraData, np.uint8).reshape((front_camera.getHeight(), front_camera.getWidth(), 4))
    # new_image1 = imageData.astype(np.uint8)
    # grayA = cv2.cvtColor(new_image1, cv2.COLOR_BGR2GRAY)
    
    # cv2.waitKey(15)
    
    # image 2
    cameraData = front_camera.getImage();
    imageData2 = np.frombuffer(cameraData, np.uint8).reshape((front_camera.getHeight(), front_camera.getWidth(), 4))
    new_image2 = imageData2.astype(np.uint8)
    # cv2.imshow("preview", new_image)
    
    
    grayB = cv2.cvtColor(new_image2, cv2.COLOR_BGR2GRAY)
    (score, diff) = compare_ssim(grayA, grayB, full=True)
    diff = (diff * 255).astype("uint8")
    
    print("reading camera image-----------", diff)
    #compare images
    
    thresh = cv2.threshold(diff, 0, 255,\
	cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    #blurring???
    blur = cv2.GaussianBlur(thresh,(5,5),0)
    ret2, th2 = cv2.threshold(blur,0,255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    canny_img = cv2.Canny(th2, 50,150)
    
    # fit the bounding box at the changes
    cnts = cv2.findContours(th2.copy(), cv2.RETR_EXTERNAL, \
    	cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    for c in cnts:
	# compute the bounding box of the contour and then draw the
	# bounding box on both input images to represent where the two
	# images differ
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(new_image1, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.rectangle(new_image2, (x, y), (x + w, y + h), (0, 0, 255), 2)
    # if not cv2.threshold == thresh:
    cv2.imwrite(r"/Users/sukanyasaha/Google Drive/Advanced-Robotics/CSCI5302-AVChallenge/photos/image_original_1_" + str(counter) + "_" + str(int(lv)) + str(int(rv)) +".jpeg", new_image1)
    # cv2.imwrite(r"/Users/sukanyasaha/Google Drive/Advanced-Robotics/CSCI5302-AVChallenge/photos/image_original_2_" + str(counter) + "_" + str(int(lv)) + str(int(rv)) +".jpeg", new_image2)
    cv2.imwrite(r"/Users/sukanyasaha/Google Drive/Advanced-Robotics/CSCI5302-AVChallenge/photos/image_thresh_"   + str(counter) + "_" + str(int(lv)) + str(int(rv)) +".jpeg", canny_img)
        # brake       = 1.
        # steer_angle = -31
        # speed       = 20 
   # if np.sum(new_image[0]) 
    # print("reading camera image-----------", thresh)
    new_image1 = new_image2
    grayA = cv2.cvtColor(new_image1, cv2.COLOR_BGR2GRAY)
    counter += 1  
    
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
    
    