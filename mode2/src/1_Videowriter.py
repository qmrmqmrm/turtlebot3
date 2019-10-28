#!/usr/bin/env python

import time
import numpy as np
import rospy
import roslib
import cv2
from sensor_msgs.msg import CompressedImage, Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError
from commonmsgs.msg import msg_mode, twist, msg_lane
bridge = CvBridge()
class robo():
	def __init__(self):
		#self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
		self.img_pub = rospy.Publisher('/image_center', Image, queue_size = 1)
	
		self.modesub = rospy.Subscriber('/mode_msg',mode_msg, self.modemsg)
		#self.twistpub = rospy.Publisher('/mode_twist', twist, queue_size=10)
		self.lanepub = rospy.Publisher('/lane_msg',msg_lane,queue_size=10)
		self.lanemsg= msg_lane()
		self.lsd = cv2.createLineSegmentDetector(0)
		#self.vel_msg = twist()
		self.cap = None
		self.frame = None
		self.hsv = None
		self.mode =None
		self.cnt = None
	
	def modemsg(self,msg):
		self.mode = msg.mode
		self.cnt = msg.cnt

	
	def keeping(self):
		crop_L = self.hsv[250:380, :220]
        	crop_R = self.frame[250:380, 402:]
        	L_mask = cv2.inRange(crop_L,(21,50,100),(36,255,255))
        	L2_mask = cv2.inRange(crop_L,(36,0,165),(255,255,255))
        	R_mask = cv2.inRange(crop_R,(230,230,230),(255,255,255))
        	R2_mask = cv2.inRange(crop_R,(21,50,100),(36,255,255))
        	img_1 = np.hstack([L_mask, R_mask])
        	img_2 = np.hstack([L2_mask, R2_mask])
        	self.img = np.vstack([img_1,img_2])
        	
		self.yello_line = self.lsd.detect(L_mask)
		self.yello_line2 = self.lsd.detect(L2_mask)
		self.white_line = self.lsd.detect(R_mask)
		self.white_line2 = self.lsd.detect(R2_mask)
		#print("ye1", self.yello_line)
		#print("ye2", self.yello_line2)
		#print("wh1", self.white_line)
		#print("wh2", self.white_line2)
		
		if self.yello_line[0] is None  :
			self.lanemsg.yellow = False
			self.lanemsg.white = True
			
		elif self.white_line[0] is None :
			self.lanemsg.yellow = True
			self.lanemsg.white = False
			
		#elif self.yello_line is None:
		#	self.lanemsg.yellow = False
		#	self.lanemsg.white = False
		#	rate = rospy.Rate(0.9)
		#	rate.sleep()
		#	self.vel_msg.linear.x = 0
		#	self.vel_msg.angular.z = 0
		#	print("stop")
	
		else:
			self.lanemsg.yellow = True
			self.lanemsg.white = True
			
		
		self.lanepub.publish(self.lanemsg)
	def imageupdata(self):
		#image = self.image_np
		self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		
def main():
	rospy.init_node("lane")
	turtle = robo()
	time.sleep(1.2)
	rate = rospy.Rate(10)
	turtle.cap = cv2.VideoCapture(2)
	if not turtle.cap.isOpened():
        	print("open fail camera")
	while not rospy.is_shutdown():
		ret, turtle.frame = turtle.cap.read()
		if ret:
			turtle.imageupdata()
			turtle.keeping()
			frame_ = bridge.cv2_to_imgmsg(turtle.frame, "bgr8")
			turtle.img_pub.publish(frame_)
		rate.sleep()

if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException:
		pass

