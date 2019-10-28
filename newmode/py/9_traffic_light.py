#!/usr/bin/env python 
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from newmode.msg import msg_traffic

import cv2

bridge = CvBridge()


class Cap:
	def __init__(self):
		self.cap = None
		self.ret = None
		self.frame = None
		self.count_ry = 0
		self.change_ry = None
        	self.count_result = 0
		self.traffic_light = True
		self.ori = None
		self.msg = msg_traffic()
		self.pub_msg = rospy.Publisher('/traffic_msg',msg_traffic, queue_size = 1)
		self.img_sub = rospy.Subscriber('/image_right', Image, self.img_msg)
	def img_msg(self,msg):
		try:
			self.frame = bridge.imgmsg_to_cv2(msg,"bgr8")
		except CvBridgeError as e:
			print(e)

	def traffic(self):
		#rospy.loginfo(self.msg)
		self.msg.mode = "traffic"
		self.ori = self.frame[0:120, 50:250]
		traffic_lower_ry = np.array([-10, 0, 0])
		traffic_upper_ry = np.array([30, 255, 255])
		change_hsv = cv2.cvtColor(self.ori, cv2.COLOR_BGR2HSV)
		change_hsv = cv2.medianBlur(change_hsv, 5)
		on_ry = cv2.inRange(change_hsv, traffic_lower_ry, traffic_upper_ry)
            	res_ry = cv2.bitwise_and(self.ori, self.ori, mask=on_ry)
		change_ry = cv2.cvtColor(res_ry, cv2.COLOR_HSV2BGR)
            	self.change_ry = cv2.cvtColor(change_ry, cv2.COLOR_BGR2GRAY)
		circles_ry = cv2.HoughCircles(self.change_ry, cv2.HOUGH_GRADIENT, 1, 15, param1=70, param2=30, minRadius=0, maxRadius=0)
		if self.count_result is 0:
			print("1")
                	if circles_ry is not None:
                		circles_ry = np.uint16(np.around(circles_ry))
                		for c_ry in circles_ry[0, :]:
                       			center_ry = (c_ry[0], c_ry[1])
                       			radius_ry = c_ry[2]
                       			if center_ry is not None and radius_ry is not None:
                       				cv2.circle(change_ry, center_ry, radius_ry, (0, 255, 0), 2)
                       				self.msg.traffic_light = True
                       				#self.count_ry += 1
                       				print("2")
        		else:
        			#if self.count_ry >20:
	    				self.msg.traffic_light = False
	    				#self.count_ry =0
	    				print("3")
	    			#pass
			
def main():
	pub = rospy.Publisher('/image_raw', Image, queue_size = 1)
        play = Cap()
	rospy.init_node("traffic_light")
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		play.traffic()
		k = cv2.waitKey(10)
		frame_ = bridge.cv2_to_imgmsg(play.change_ry, "mono8")
		pub.publish(frame_)
                play.pub_msg.publish(play.msg)
                if play.msg.traffic_light == False:
                	return 0
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

