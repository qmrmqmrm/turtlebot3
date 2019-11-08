#!/usr/bin/env python
import rospy
from newmode.msg import mode_msg, twist, msg_lane
import numpy as np

class test():
	def __init__(self):
		
		self.lanesub = rospy.Subscriber('/lane_msg', msg_lane, self.lanemsg)
		self.modesub = rospy.Subscriber('/mode_msg',mode_msg, self.modemsg)
		self.twistpub = rospy.Publisher('/mode_twist', twist, queue_size=1)
		self.vel_msg = twist()
		self.mode = None
		self.lane_yellow = None
		self.lane_white= None
		self.count = 0

	def modemsg(self,msg):
		self.mode = msg.mode
		self.cnt = msg.cnt

	def lanemsg(self,a):
		self.lane_yellow = a.yellow
		self.lane_white= a.white
	def lane_go(self):
		
			
			if self.lane_yellow == False and self.lane_white == True:
				self.vel_msg.linear.x = 0.128
				self.vel_msg.angular.z = 0.44
				print("turn_l")
			if self.lane_yellow == True and self.lane_white == False:
				self.vel_msg.linear.x = 0.128
				self.vel_msg.angular.z = -0.44
			if self.lane_yellow == True and self.lane_white == True:
				self.vel_msg.linear.x = 0.15
				self.vel_msg.angular.z = 0
				print("go")
			self.twistpub.publish(self.vel_msg)
			


def main():
	rospy.init_node('lane_move')
	te = test()
	while not rospy.is_shutdown():
		if te.mode == 0:
			te.lane_go()
		rospy.sleep(0.1)
     
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass





