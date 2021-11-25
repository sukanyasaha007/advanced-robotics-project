#!/home/m_dyse/pyenvs/slam_env/bin/python3
import sys
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, LaserScan, Imu
from tesla_controller.msg import sign as sign_msg
from webots_ros.srv import set_float

class TeslaController:
	def __init__(self):
		# new stuff
		self.KP = 0.25
		self.KI = 0.006
		self.KD = 2
		self.PID_need_reset = False
		self.FILTER_SIZE = 3
		self.UNKNOWN = 99999.99
		self.sick_width = 180

		self.angle = 0
		self.obstacle_angle = 0

		# ROS stuff
		self.bridge = CvBridge()
		self.sign_sub = rospy.Subscriber("/sign_detection", sign_msg, self.sign_callback)
		self.cam_sub = rospy.Subscriber("/tesla/front_camera/image", Image, self.cam_callback)
		self.lidar_sub = rospy.Subscriber("/tesla/Sick_LMS_291/laser_scan/layer0", LaserScan, self.lidar_callback)
		self.imu_sub = rospy.Subscriber("/IMU/roll_pitch_yaw", Imu, self.imu_callback)
		self.accel_sub = rospy.Subscriber("/accelerometer/values", Imu, self.accel_callback)
		self.pose_pub = rospy.Publisher("/tesla/pose", Float64MultiArray, queue_size=15)

		# old stuff
		self.cv_image = None
		self.speed = 25
		self.steering_angle = 0.0
		self.t = 0.004
		self.pose = np.array([0,0,0]).astype(np.float64)
		self.veloctiy = np.array([0,0,0]).astype(np.float64)
		self.orientation = np.array([0,0,0]).astype(np.float64)

	def sign_callback(self, data):
		'''
		see sign?
			stop sgn: stop; wait; go
			red light: stop
			green light: go
		'''
		pass

	def cam_callback(self, ros_image):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.angle = self.filter_angle(self.process_image(cv_image))

		#self.controller_callback()

	def lidar_callback(self, lidar_message):
		sick_data = np.array(lidar_message.ranges)
		obstacle_dist = 0
		self.obstacle_angle = self.process_sick_data(sick_data)

		#self.controller_callback()


	def controller_callback(self):
		if self.obstacle_angle != self.UNKNOWN:
			obstacle_steering = self.steering_angle

			if self.obstacle_angle > 0.0 and self.obstacle_angle < 0.4:
				obstacle_steering = self.steering_angle + (self.obstacle_angle - 0.25) / self.obstacle_dist
			elif self.obstacle_angle > -0.4:
				obstacle_steering = self.steering_angle + (self.obstacle_angle + 0.25) / self.obstacle_dist

			steer = float(self.steering_angle)

			if self.angle != self.UNKNOWN:
				line_following_steering = self.apply_pid(self.angle)
				if obstacle_steering > 0 and line_following_steering > 0:
					steer = obstacle_steering if obstacle_steering > line_following_steering else line_following_steering
				elif obstacle_steering < 0 and line_following_steering < 0:
					steer = obstacle_steering if obstacle_steering < line_following_steering else line_following_steering
				else:
					self.PID_need_reset = True

				self.set_steering_angle(steer)
		elif self.angle != self.UNKNOWN:
			self.set_steering_angle(self.apply_pid(self.angle))
		else:
			self.PID_need_reset = True



	def imu_callback(self, imu_message):
		self.orientation = np.array(imu_message.orientation)

	def accel_callback(self, accel_message):
		accel = np.array(accel_message.linear_acceleration)
		accel = self.R(accel, self.orientation)
		self.pose, self.velocity = self.accel2pose(self.pose, self.velocity, accel, self.t)

	def spin(self):
		pose = Float64MultiArray()
		pose.data = np.concatenate([self.pose, self.orientation])
		self.pose_pub.publish(pose)
		#self.h = np.max(np.min(sigmoid((self.h0 * (1 - self.h1_weight)) + (self.h1 * self.h1_weight)), 0.9), 0.0001)
		#self.set_steer_angle(self.h)
		#rospy.loginfo(f'Heading {self.h, self.h1_weight}')
		self.controller_callback()
		self.set_speed(self.speed)

	def R(self, pose, ref):
	    R = np.matrix([
	        [np.cos(ref[1])*np.cos(ref[2]),
	        np.cos(ref[1])*np.sin(ref[2]),
	        -np.sin(ref[1])],
	        [np.sin(ref[0])*np.sin(ref[1])*np.cos(ref[2])-(np.cos(ref[0])*np.sin(ref[2])),
	        np.sin(ref[0])*np.sin(ref[1])*np.sin(ref[2])+(np.cos(ref[0])*np.cos(ref[2])),
	        np.sin(ref[0])*np.cos(ref[1])],
	        [np.cos(ref[0])*np.sin(ref[1])*np.cos(ref[2])+(np.sin(ref[0])*np.sin(ref[2])),
	        np.cos(ref[0])*np.sin(ref[1])*np.sin(ref[2])-(np.sin(ref[0])*np.cos(ref[2])),
	        np.cos(ref[0])*np.cos(ref[1])]])

	    return np.array(np.matmul(R, pose.T))[0]

	def accel2pose(self, pose, vel, accel, t):
	    pose += vel * t + (accel * t**2 / 2)
	    vel += accel * t
	    return pose, vel

	def set_speed(self, speed):
		rospy.wait_for_service('/tesla/automobile/set_cruising_speed')
		srv = rospy.ServiceProxy('/tesla/automobile/set_cruising_speed', set_float)
		return_msg = srv(speed)

	def set_steer_angle(self, angle):
		rospy.wait_for_service('/tesla/automobile/set_steering_angle')
		srv = rospy.ServiceProxy('/tesla/automobile/set_steering_angle', set_float)
		return_msg = srv(angle)

	def process_image(self,img):
		fov = 1.0

		h,w,_= img.shape

		cropped = img[int(h * 0.8):, int(0.31 * w): int(0.69 * w)]
		h,w,_ = cropped.shape

		lower_white = np.array([110,110,110])
		upper_white = np.array([255,255,255])

		bin_img = cv2.inRange(cropped, lower_white, upper_white)

		_, points = np.nonzero(bin_img)

		num_pixels = h * w
		sum_x = 0
		pixel_count = 0

		if len(points) > 0:
			for x in points[0]:
				sum_x += x % w
				pixel_count += 1

			return (float(sum_x) / float(pixel_count) / camera_width - 0.5) * fov

		else:
			return self.UNKNOWN


	def filter_angle(self, new_value):
		first_call = True
		old_value = np.zeros(self.FILTER_SIZE)

		i = 0

		if first_call or new_value == self.UNKNOWN:
			first_call = False
			for i in range(self.FILTER_SIZE):
				old_value[i] = 0.0
		else:
			for i in range(self.FILTER_SIZE - 1):
				old_value[i] = old_value[i+1]

		if new_value == self.UNKNOWN:
			return self.UNKNOWN
		else:
			old_value[self.FILTER_SIZE - 1] = new_value
			sum = 0.0
			for i in range(self.FILTER_SIZE):
				sum += old_value[i]
			return float(sum) / float(self.FILTER_SIZE)


	def process_sick_data(self, sick_data):
		sick_fov = np.pi
		HALF_AREA = 20
		collision_count = 0
		sum_x = 0
		x = 0
		self.obstacle_dist = 0

		for x in range(self.sick_width // 2 -HALF_AREA,
					   self.sick_width // 2 + HALF_AREA):
			range = sick_data[x]
			if range < 20.0:
				sum_x += x
				collision_count += 1
				self.obstacle_dist += range

		if collision_count == 0:
			return self.UNKNOWN

		self.obstacle_dist = self.obstacle_dist / collision_count
		return (float(sum_x) / collision_count / sick_width - 0.5) * sick_fov

	def apply_pid(angle):
		old_value = 0.0
		integral = 0.0

		if self.PID_need_reset:
			old_value = angle
			integral = 0.0
			self.PID_need_reset = False

		if np.sign(angle) != np.sign(old_value):
			integral = 0.0

		diff = angle - old_value

		if integral < 30 and integral > -30:
			integral += angle

		old_value = angle
		return self.KP * yellow_line_angle + self.KI * integral + self.KD * diff

'''
class PID:
	def __init__(self, p, i, d):
		self.kp = 0.25
		self.ki = 0.006
		self.kd = 2
		self.p = 0
		self.i = 0
		self.d = 0
	def converge(self, error):
		self.i += error
		self.d = self.p - error
		self.p = error
		return (self.p * self.kp) + (self.i * self.ki) + (self.d * self.kd)
def sigmoid(z):
	return 2.0 / (1.0 + np.exp(-z)) - 1.0
def R(pose, ref):
    R = np.matrix([
        [np.cos(ref[1])*np.cos(ref[2]),
        np.cos(ref[1])*np.sin(ref[2]),
        -np.sin(ref[1])],
        [np.sin(ref[0])*np.sin(ref[1])*np.cos(ref[2])-(np.cos(ref[0])*np.sin(ref[2])),
        np.sin(ref[0])*np.sin(ref[1])*np.sin(ref[2])+(np.cos(ref[0])*np.cos(ref[2])),
        np.sin(ref[0])*np.cos(ref[1])],
        [np.cos(ref[0])*np.sin(ref[1])*np.cos(ref[2])+(np.sin(ref[0])*np.sin(ref[2])),
        np.cos(ref[0])*np.sin(ref[1])*np.sin(ref[2])-(np.sin(ref[0])*np.cos(ref[2])),
        np.cos(ref[0])*np.cos(ref[1])]])
    return np.array(np.matmul(R, pose.T))[0]
def accel2pose(pose, vel, accel, t):
    pose += vel * t + (accel * t**2 / 2)
    vel += accel * t
    return pose, vel
class TeslaController:
	def __init__(self):
		self.bridge = CvBridge()
		self.sign_sub = rospy.Subscriber("/sign_detection", sign_msg, self.sign_callback)
		self.cam_sub = rospy.Subscriber("/tesla/front_camera/image", Image, self.cam_callback)
		self.lidar_sub = rospy.Subscriber("/tesla/Sick_LMS_291/laser_scan/layer0", LaserScan, self.lidar_callback)
		self.imu_sub = rospy.Subscriber("/IMU/roll_pitch_yaw", Imu, self.imu_callback)
		self.accel_sub = rospy.Subscriber("/accelerometer/values", Imu, self.accel_callback)
		self.pose_pub = rospy.Publisher("/tesla/pose", Float64MultiArray, queue_size=15)
		self.sign_distance = None
		self.sign_seen = None
		self.cv_image = None
		self.speed = 20
		self.steering_angle = 0.0
		self.h0 = 0.0
		self.h1 = 0.0
		self.h1_weight = 0.0
		self.h = 0.0
		self.t = 0.004
		self.pose = np.array([0,0,0]).astype(np.float64)
		self.veloctiy = np.array([0,0,0]).astype(np.float64)
		self.orientation = np.array([0,0,0]).astype(np.float64)
		self.pid_cv = PID(0.015, 0.00, 0.00)
		self.pid_li = PID(0.225, 0.0, 0.15)
	def spin(self):
		pose = Float64MultiArray()
		pose.data = np.concatenate([self.pose, self.orientation])
		self.pose_pub.publish(pose)
		self.h = np.max(np.min(sigmoid((self.h0 * (1 - self.h1_weight)) + (self.h1 * self.h1_weight)), 0.9), 0.0001)
		self.set_steer_angle(self.h)
		rospy.loginfo(f'Heading {self.h, self.h1_weight}')
		self.set_speed(self.speed)
	def set_speed(self, speed):
		rospy.wait_for_service('/tesla/automobile/set_cruising_speed')
		srv = rospy.ServiceProxy('/tesla/automobile/set_cruising_speed', set_float)
		return_msg = srv(speed)
	def set_steer_angle(self, angle):
		rospy.wait_for_service('/tesla/automobile/set_steering_angle')
		srv = rospy.ServiceProxy('/tesla/automobile/set_steering_angle', set_float)
		return_msg = srv(angle)
	def sign_callback(self, sign_message):
    	data = sign_message.data
    	data_ = data.split(",")
    	self.sign_distance = int(data_[0])
    	self.sign_seen = int(data_[1])
	def lidar_callback(self, lidar_message):
		rangeImage = np.array(lidar_message.ranges)
		max_lidar_10 = 800.0
		left = np.sum(rangeImage[:10])
		right = np.sum(rangeImage[-10:])
		self.h1 = self.pid_li.converge(right - left)
		left_weight = 1 - (left/max_lidar_10)
		right_weight = 1 - (right / max_lidar_10)
		self.h1_weight = (left_weight + right_weight) / 2.0
	def cam_callback(self, ros_image):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
		except CvBridgeError as e:
			print(e)
		self.h0 += self.pid_cv.converge(process_image(self.cv_image))
		#rospy.loginfo(f'Heading {self.h0, sigmoid(self.h0)}')
	def imu_callback(self, imu_message):
		self.orientation = np.array(imu_message.orientation)
	def accel_callback(self, accel_message):
		accel = np.array(accel_message.linear_acceleration)
		accel = R(accel, self.orientation)
		self.pose, self.velocity = accel2pose(self.pose, self.velocity, accel, self.t)
'''

if __name__ == '__main__':
	rospy.init_node('tesla_driver', anonymous=False)
	driver = TeslaController()
	while not rospy.is_shutdown():
		driver.spin()
