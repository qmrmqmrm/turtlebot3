#!/usr/bin/env python 

import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

bridge = CvBridge()
class cap:
	def __init__(self):
		#self.cap_center = None
		#self.cap_right = None
		self.cap_left = None
		#self.ret_cetner = None
		#self.ret_right = None
		self.ret_left = None
		#self.frame_center = None
		#self.frame_right = None
		self.frame_left = None
		#self.pub_right = rospy.Publisher('/image_right', Image, queue_size=1)
		self.pub_left = rospy.Publisher('/image_left', Image, queue_size=1)

def main() :
	cam = cap()	
	rospy.init_node('cam')
	rate = rospy.Rate(10)

	#cam.cap_right = cv2.VideoCapture(2)
	cam.cap_left = cv2.VideoCapture(0)
	if (cam.cap_left.isOpened() == False): 
		print("Unable to read camera feed")
	else:
		#cam.cap_right.set(3, 320)
		#cam.cap_right.set(4, 480)
		cam.cap_left.set(3, 320)
		cam.cap_left.set(4, 480)

	while not rospy.is_shutdown():
		#cam.ret_right, cam.frame_right = cam.cap_right.read()
		cam.ret_left, cam.frame_left = cam.cap_left.read()
		#cv2.imshow("jjo", cam.frame_right)
		if (cam.ret_left == True): 
			#frame_right_ = bridge.cv2_to_imgmsg(cam.frame_right, "bgr8")
			frame_left_ = bridge.cv2_to_imgmsg(cam.frame_left, "bgr8")
			#cam.pub_right.publish(frame_right_)
			cam.pub_left.publish(frame_left_)
		rate.sleep()

if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException:
		pass
