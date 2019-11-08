#!/usr/bin/env python
import rospy
from newmode.msg import msg_laser, twist,mode_msg
from sensor_msgs.msg import LaserScan
import numpy as np



class laser:
	def __init__(self):
		self.mode = None
		self.a = 0
		self.vel_tw = twist()
		self.lasermsg = msg_laser()
		self.avg = 0
		self.sum_r = 0
		self.sum_l = 0
		self.count_r = 0
		self.count_l = 0
		self.twistpub = rospy.Publisher("/mode_twist", twist, queue_size=1)
		self.laserpub = rospy.Publisher('/msg_laser',msg_laser, queue_size =1)
		self.modesub = rospy.Subscriber('/mode_msg',mode_msg, self.modemsg)
		self.mode_sub = None
		self.cnt_sub = None
		
	def modemsg(self,msg):
		self.mode_sub = msg.mode
		self.cnt_sub = msg.cnt
		#print("laser",msg.mode,msg.cnt)
		
		
	

	def turn(self, a):
		self.vel_tw.linear.x = 0
		self.vel_tw.angular.z = a*(np.pi)/6
		self.twistpub.publish(self.vel_tw)

	def stop(self):
		self.vel_tw.linear.x = 0
		self.vel_tw.angular.z = 0
		self.twistpub.publish(self.vel_tw)
		
	def callback(self, data):
		if self.mode_sub !=3:
			self.laserpub.publish(self.lasermsg)
			for num_r in data.ranges[1:40]:
				if num_r > 0.02 and num_r < 0.5:		
				#checking laser value
					self.sum_r = self.sum_r + num_r
					self.count_r = self.count_r + 1
				

			for num_l in data.ranges[310:340]:
				if num_l >0.02 and num_l < 0.5:
					self.sum_l = self.sum_l + num_l
					self.count_l = self.count_l + 1
					#self.laserpub.publish(self.lasermsg)

			if self.count_r > self.count_l:
				self.avg = self.sum_r / self.count_r
				if self.avg < 0.28:
					#if self.mode_sub == 2				
					#average laser value
					self.turn(-1)
					self.mode = "turn_r"
					print(self.mode)
					self.lasermsg.bool = True		
				
				else:
					#if self.mode_sub == 2 and self.cnt_sub  == 0:				
					#self.turn(0)
					#self.gos()
					self.mode = "gos"
					print(self.mode)
					self.lasermsg.bool = False
				
			
			elif self.count_l > self.count_r:
				self.avg = self.sum_l / self.count_l
				if self.avg < 0.3:
					#if self.mode_sub == 2 and self.cnt_sub  == 0:					
					self.turn(1)
					self.mode = "turn_l"
					print(self.mode)
					self.lasermsg.bool = True
				
			
				else:
					#if self.mode_sub == 2 and self.cnt_sub  == 0:				
					#self.turn(0)
					#self.gos()
					self.mode = "gos"
					print(self.mode)
					self.lasermsg.bool = False
				
			else:
				#if self.mode_sub == 2 and self.cnt_sub == 0:				
				#self.turn(0)
				#self.gos()
				self.mode = "gos"
				print(self.mode)
				self.lasermsg.bool = False
			
			self.sum_r = 0
			self.count_r = 0
			self.sum_l = 0
			self.count_l = 0
			

		
def main():
	move = laser()
	rospy.init_node("laser_move")
	rospy.Subscriber("/scan", LaserScan, move.callback)
	
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		move.a += 1
			
	rate.sleep()

if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException:
		pass

