#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from newmode.msg import msg_lane, msg_sign
class T():
	def __init__(self):
		self.lanepub = rospy.Publisher('/lane_msg', msg_lane, queue_size=1)
		self.signsub = rospy.Subscriber('/sign_msg',msg_sign, self.signmsg)
		self.lanemsg = msg_lane()
		self.sign = None
		self.img_sub = rospy.Subscriber('/image_raw_right', Image,self.img_msg)
	
	def img_msg(self,msg):
		try:
			self.frame = bridge.imgmsg_to_cv2(msg,"bgr8")
		except CvBridgeError as e:
			print(e)
			
	def signmsd(msg) : 
		self.sign = msg.sign
		
	def two_way(self):
		


		#if self.sign is not None:
			#sign_1= self.sign

if __name__ == '__main__':
	rospy.init_node('twist_ch')
	try:
		t =T()
		t.main()
	except rospy.ROSInterruptException:	pass




