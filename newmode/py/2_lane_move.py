#!/usr/bin/env python
import rospy
from newmode.msg import mode_msg, twist, msg_lane
import numpy as np

class test():
	def __init__(self):
		
		self.lanesub = rospy.Subscriber('/lane_msg', msg_lane, self.lanemsg)
		self.modesub = rospy.Subscriber('/mode_msg',mode_msg, self.modemsg)
		self.twistpub = rospy.Publisher('/mode_twist', twist, queue_size=10)
		self.vel_msg = twist()
		self.mode = None
		self.lane_yellow = None
		self.lane_white= None
		self.count = 0

	def modemsg(self,msg):
		self.mode = msg.mode

	def lanemsg(self,a):
		self.lane_yellow = a.yellow
		self.lane_white= a.white
	def lane_go(self):
		if self.mode == 0:
			
			if self.lane_yellow ==False and self.lane_white ==True:
				self.vel_msg.linear.x = 0.138
				self.vel_msg.angular.z = 0.395
				print("turn_l")
			if self.lane_yellow ==True and self.lane_white ==False:
				self.vel_msg.linear.x = 0.138
				self.vel_msg.angular.z = -0.395
			if self.lane_yellow == True and self.lane_white ==True:
				self.vel_msg.linear.x = 0.2
				self.vel_msg.angular.z = 0
				print("go")
			self.twistpub.publish(self.vel_msg)
			


def main():
	rospy.init_node('lane_move')
	te = test()
	while not rospy.is_shutdown():
		te.lane_go()
		rospy.sleep(0.1)
     
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass





	#		if self.new_angle == -90:
	#			self.vel_tw.linear.x = 0.18
	#			self.vel_tw.angular.z = 0
	#		elif self.new_angle <=-100 and self.new_angle > -120 :
	#			self.vel_tw.linear.x = 0.14
	#			self.vel_tw.angular.z = 0.45
	#		elif self.new_angle <= -135 and self.new_angle > -150:
	#			self.vel_tw.linear.x = 0.18
	#			self.vel_tw.angular.z = 0
	#		elif (self.new_angle >=-85 and self.new_angle <= -79) or (self.new_angle>123 and self.new_angle <135):
	#			self.vel_tw.linear.x = 0.14
	#			self.vel_tw.angular.z = -0.45
	#			
	#		elif (self.new_angle <-150 and self.new_angle >-165) :
	#			self.vel_tw.linear.x = 0.18
	#			self.vel_tw.angular.z = 0
	#		elif self.new_angle <= -165 and self.new_angle > -175:
	#			print(self.new_angle)
	#			self.vel_tw.angular.z = 0
	#			self.twistpub.publish(self.vel_tw)
	#			rate = rospy.Rate(0.312)
	#			rate.sleep()
	#			self.vel_tw.linear.x = 0.18
	#			self.vel_tw.angular.z = 1
	#			#self.count = self.count + 1
	#		else :
	#			self.vel_tw.linear.x = 0.18
	#			self.vel_tw.angular.z = 0
	#			
	#		print(self.vel_tw)
	#		print("he")
#
			#if self.count >= 20 and self.count<=30:
			#	self.vel_tw.linear.x = 0.15
			#	self.vel_tw.angular.z = 0.5
			#	self.count = self.count - 0.25
			#else:
			#	self.vel_tw.linear.x = 0.18
			#	self.vel_tw.angular.z = 0
			
