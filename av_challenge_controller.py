
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
    
    """
    #remove shadows
    rgb_planes = cv2.split(frame)
    
    result_planes = []
    result_norm_planes = []
    for plane in rgb_planes:
        dilated_img = cv2.dilate(plane, np.ones((7,7), np.uint8))
        bg_img = cv2.medianBlur(dilated_img, 21)
        diff_img = 255 - cv2.absdiff(plane, bg_img)
        norm_img = cv2.normalize(diff_img,None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
        result_planes.append(diff_img)
        result_norm_planes.append(norm_img)
    
    # result = cv2.merge(result_planes)
    result_norm = cv2.merge(result_norm_planes)
    cv2.imshow("mask", result_norm)
    """
    
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
        # if np.sum(res[front_camera.getHeight()-6][0]) == 0 and np.sum(res) < 48000:
            # print("for 31", np.sum(res))
            # brake       = 1.
            # steer_angle = 28
            # speed       = 8
        if np.sum(res[front_camera.getHeight()-6][0]) == 0 :
            print("for -28", np.sum(res))
            brake       = 1.
            steer_angle = -28
            speed       = 8
        elif np.sum(res[front_camera.getHeight()-6][127])==0:
            print("for 28", np.sum(res))
            brake       = 1.
            steer_angle = 27
            speed       = 8
        else:
            print("else", np.sum(res))
            brake       = 0.0    
            steer_angle = 0
            speed       = 8
    else:
        if lv > 3.7 and rv > 3.7:
            brake       = 0.0    
            steer_angle = 0
            speed       = 35
        elif lv < 3.7:
            brake       = 1.
            steer_angle = -31
            speed       = 20      
        elif rv < 3.7:
            brake       = 1.
            steer_angle = 31
            speed       = 15
    
    robot.setBrakeIntensity(brake)
    robot.setSteeringAngle(steer_angle)
    robot.setCruisingSpeed(speed)
    
    
    """av_challenge_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
"""
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


def get_canny_image(iamge):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny


def get_region_of_interest(image):
    height = image.shape[0]
    width = image.shape[1]
    mask = np.zeros_like(image)

    # this the triage of the road
    
    polygons = np.array([[(0, 54),  (127, 53), (57, 37), (63, 36)]], np.int32)

    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


image = cv2.imread(r"image_original_2_469_43.jpeg")
lane_image = np.copy(image)
image_canny = canny(lane_image)
image_polygon = region_of_interest(image_canny)
# plt.imshow(image_canny)
plt.imshow(image_polygon)
plt.show()

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
    
    
    # grayB = cv2.cvtColor(new_image2, cv2.COLOR_BGR2GRAY)
    # (score, diff) = compare_ssim(grayA, grayB, full=True)
    # diff = (diff * 255).astype("uint8")
    
    # print("reading camera image-----------", diff)
    #compare images
    
    # thresh = cv2.threshold(diff, 0, 255,\
	# cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    #blurring???
    # blur = cv2.GaussianBlur(thresh,(5,5),0)
    # ret2, th2 = cv2.threshold(blur,0,255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # canny_img = cv2.Canny(th2, 50,150)
    
    # fit the bounding box at the changes
    # cnts = cv2.findContours(th2.copy(), cv2.RETR_EXTERNAL, \
    	# cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)
    
    # for c in cnts:
	# compute the bounding box of the contour and then draw the
	# bounding box on both input images to represent where the two
	# images differ
        # (x, y, w, h) = cv2.boundingRect(c)
        # cv2.rectangle(new_image1, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # cv2.rectangle(new_image2, (x, y), (x + w, y + h), (0, 0, 255), 2)
    # if not cv2.threshold == thresh:
    # cv2.imwrite(r"/Users/sukanyasaha/Google Drive/Advanced-Robotics/CSCI5302-AVChallenge/photos/image_original_1_" + str(counter) + "_" + str(int(lv)) + str(int(rv)) +".jpeg", new_image1)
    
    
    image_canny = get_canny_image(lane_image)
    image_polygon = region_of_interest(image_canny)
    # plt.imshow(image_canny)
    plt.imshow(image_polygon)
    plt.show()
    cv2.imwrite(r"/Users/sukanyasaha/Google Drive/Advanced-Robotics/CSCI5302-AVChallenge/photos/image_original_2_" + str(counter) + "_" + str(int(lv)) + str(int(rv)) +".jpeg", new_image2)
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
    
"""
