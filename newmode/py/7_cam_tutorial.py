#!/usr/bin/env python 
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from newmode.msg import msg_detect

import cv2

bridge = CvBridge()
msg = msg_detect()

class Cap:
	def __init__(self):
		self.cap = None
		self.ret = None
		self.frame = None
		self.res = None
		self.area = None
		self.red_count = 0
		self.img_sub = rospy.Subscriber('/image_left', Image,self.img_msg)
	def img_msg(self,msg):
		try:
			self.frame = bridge.imgmsg_to_cv2(msg,"bgr8")
		except CvBridgeError as e:
			print(e)
		
	def red_img(self):
           	change_hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            	lower_red = np.array([-10, 100, 100])
            	upper_red = np.array([10, 255, 255])
		kernel = np.ones((5,5), np.uint8)
            	roi = change_hsv[70:180, 0:320]
            	roi = cv2.morphologyEx(roi, cv2.MORPH_OPEN, kernel, iterations=2)
            	on_v = cv2.inRange(roi, lower_red, upper_red)
            	res = cv2.bitwise_and(roi, roi, mask=on_v)
            	self.res = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)

	def cont(self):
        	im_gray = cv2.cvtColor(self.res, cv2.COLOR_BGR2GRAY)
        	image, contours, hie = cv2.findContours(im_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        	for cnt in contours:
            		self.area = cv2.contourArea(cnt)
            		if (self.area >= 500):
                		self.red_count += 1
                		#print(self.red_count)
        			#if self.red_count > 30:
                    			#print(self.red_count)
            		elif (self.area >= 150) and (self.area < 500):
                		self.red_count = 0
                		#print(self.red_count)
            		if self.red_count >= 30:
                		msg.mode = "track_bar"
				msg.bar = True
				rospy.loginfo("%s" % (msg.bar))
				rospy.loginfo("%s" % (msg.mode))
				
			print(self.area , self.red_count)
		if msg.mode == "track_bar":
			self.red_count -= 1
			if self.red_count <=0:
				self.red_count = 0
				msg.bar = False
				rospy.loginfo("%s" % (msg.bar))

def main():
	pub = rospy.Publisher('/image_det', Image, queue_size = 1)
	play = Cap()
        pub_det = rospy.Publisher('/det_msg',msg_detect, queue_size = 1)
	rospy.init_node("cam_tutorial")
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		play.red_img()
		play.cont()
		k = cv2.waitKey(10)
		frame_ = bridge.cv2_to_imgmsg(play.res, "bgr8")
		pub.publish(frame_)
                pub_det.publish(msg)
		if k == 27:
		    play.cap.release()
		    cv2.destroyAllWindows()
		    break
	rate.sleep()


if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException:
		pass
