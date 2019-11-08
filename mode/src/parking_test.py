#!/usr/bin/env python 
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from newmode.msg import mode_msg, twist, msg_park
import lanemove


class Park:
	def __init__(self):
		
		
		self.vel_tw =twist()
		self.twistpub = rospy.Publisher('/mode_twist', twist, queue_size=10)
		self.modesub = rospy.Subscriber('/mode_msg', mode_msg, self.modemsg)
		self.lasersub = rospy.Subscriber("/scan", LaserScan, self.callback)
		self.pubpark  =rospy.Publisher('/parkend',msg_park, queue_size = 10)
		#self.sub = rospy.Subscriber('delay_pub',Bool,self.bool)
		#self.pub = rospy.Publisher('delay_sub',Int32,queue_size =1)
		self.park_end = msg_park()
		self.park_end.park = 0
		#self.delay = Int32()
		self.mode = None
		self.cnt = None
		self.laser_range = []
		self.avg = 0
		self.sum_r = 0
		self.count_r = 0
		self.lane = lanemove.test()
		#self.delay.data = 0
		self.count = 0
		self.bool = False
		
	
		
	def modemsg(self,d):
		self.mode = d.mode
		self.cnt = d.cnt
		#print(self.mode)
	
	def callback(self, data):
		self.laser_range = data.ranges
		#print(self.laser_range)
	
	
	def parking(self, a):
		self.turn(a)
		self.sleep(0.33333)
		self.gos(1)
		print("in")
		self.sleep(0.33)
		self.stop()
		self.sleep(0.5)
		self.gos(-1)
		print("out")
		self.sleep(0.33)
		self.turn(-a)
		self.sleep(0.3333)
		self.stop()
		print("parking fin")
		self.sleep(0.5)
		self.park_end.park = 1
		#self.sleep(1)
		self.pubpark.publish(self.park_end)
		self.sleep(1)
		print("1",self.park_end)
		
		
	def sleep(self, a):
		
		b = a
		rate = rospy.Rate(b)
		rate.sleep()
				
		
	def gos(self,a):
		self.vel_tw.linear.x =a*0.1
		self.vel_tw.angular.z = 0
		self.twistpub.publish(self.vel_tw)

	def turn(self, a):
		self.vel_tw.linear.x = 0
		self.vel_tw.angular.z = a * (np.pi)/6
		self.twistpub.publish(self.vel_tw)
		#rate = rospy.Rate(1)
		#rate.sleep()

	def stop(self):
		self.vel_tw.linear.x = 0
		self.vel_tw.angular.z = 0
		self.twistpub.publish(self.vel_tw)
		
		
		
	def lasercut(self):
		print("data")
		#self.park_end.park = 0
		#self.pubpark.publish(self.park_end)
		print("start")
		for a in range(375):
			self.lane.lane_go()
			self.sleep(100)
		
		
		
		#self.stop()
		#self.sleep(0.2)
		
		
		for num_r in self.laser_range[250:320]:
			print("check")
			if num_r >= 0.02 and num_r <= 0.3:
				self.sum_r = self.sum_r + num_r
				self.count_r = self.count_r + 1
		try:
			self.avg = self.sum_r / self.count_r
		except ZeroDivisionError: self.avg =1
		print(self.avg)
		if self.avg > 0.3:
			print(">0.3")		
			self.parking(-1)
			
			
		else :
			pass
			
			#for a in range(5):
				#self.lane.lane_go()
				#self.sleep(20)
		print("fin")
		self.count += 1
		print(self.count)
		print("2",self.park_end)
		self.park_end.park = 0
		self.pubpark.publish(self.park_end)
		print("3",self.park_end)
		self.sum_r = 0
		self.count_r = 0
		rate = rospy.Rate(1)
		rate.sleep()
								
		
		
			
def main():
	rospy.init_node("park")
	park = Park()
	
	rate = rospy.Rate(10)
	
	
	while not rospy.is_shutdown():
		if park.mode == 3:
			park.lasercut()
		
		
		rate.sleep()
	
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
		
