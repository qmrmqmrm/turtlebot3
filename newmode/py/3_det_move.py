#!/usr/bin/env python
import rospy
import sys, os
from newmode.msg import mode_msg, twist, msg_detect
class det():
	def __init__(self):
		self.twistpub = rospy.Publisher('/mode_twist', twist, queue_size=10)
		self.modesub = rospy.Subscriber('/mode_msg', mode_msg, self.modemsg)

		self.vel_tw = twist()

		self.mode = None
		self.cnt = None
		self.co = None


	def modemsg(self,d):
		self.mode = d.mode
		self.cnt = d.cnt 
		
	def detmove(self):
		if self.mode == 1:
			self.vel_tw.linear.x = 0
			self.vel_tw.angular.z = 0
			self.twistpub.publish(self.vel_tw)
			
			if self.cnt == 2 :
				self.co = 1
				print("1",self.co)
		
		
			
def main():
	rospy.init_node('det_move')
	stop = det()
	while not rospy.is_shutdown():
		stop.detmove()
		rospy.sleep(0.1)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

