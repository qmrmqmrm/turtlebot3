#!/usr/bin/env python
import rospy
import sys, os
#from sensor_msgs.msg import LaserScan
from newmode.msg import mode_msg, start, msg_lane, msg_detect ,msg_laser,msg_sign_l,msg_sign_r ,twist,msg_traffic



class test():
	def __init__(self):
		#init


		self.angle = None#float
		self.bar = False
		self.traffic_light = False
		self.lane_yellow = False
		self.lane_white = False
		self.laser_bool = False

		self.signr = None
		self.signl = None
		#subscriber
		#self.startsub = rospy.Subscriber('/start',start,self.starting)
		self.lanesub = rospy.Subscriber('/lane_msg', msg_lane, self.lanemsg)
		self.lasersub = rospy.Subscriber('/msg_laser',msg_laser, self.lasermsg)
		self.detectsub = rospy.Subscriber('/det_msg', msg_detect, self.detectmsg)
		self.detectsub = rospy.Subscriber('/traffic_msg', msg_traffic, self.trafficmsg)
		self.signsubl = rospy.Subscriber('/sign_msg_l', msg_sign_l, self.signmsgl)
		self.signsubr = rospy.Subscriber('/sign_msg_r', msg_sign_r, self.signmsgr)
		#publshing
		self.mode = mode_msg()
		self.vel_tw = twist()
		self.modepub = rospy.Publisher('/mode_msg',mode_msg,queue_size=10)
		self.twistpub = rospy.Publisher('/mode_twist', twist, queue_size=10)
		self.sign_numr= None
		self.sign_numl= None
		self.sign_numr2= None
		self.sign_numl2= None
	
	def modech(self):
		#print("mode,lane",self.mode.mode,self.lane)
		if self.lane_yellow is True or self.lane_white is True:
			#print("1")
			#if self.sign_numl == 0 and self.sign_numr == 0:
			#mode=1,det use
			if self.bar == True:
				self.mode.mode = 1
				self.mode.cnt = 2
				#self.modepub.publish(self.mode)
			#elif self.bar == False:
				print("11")
				#self.modepub.publish(self.mode)
				
		    
			elif self.traffic_light == True:
				self.mode.mode = 1
				self.mode.cnt =1
				
				#self.modepub.publish(self.mode)
			#elif self.traffic_light ==False:
				print("22")
				#self.modepub.publish(self.mode)
				
			#elif self.signr == 'AVOI':
			elif self.laser_bool == True and self.bar == False and self.traffic_light == False:
				self.mode.mode =2
				self.mode.cnt =0
				#self.modepub.publish(self.mode)
			#else:
				#self.mode.mode = 0
				#self.mode.cnt = 0
			elif self.sign_numl[2] != 0:
				self.mode.mode = 6
				self.mode.cnt = 0
				self.vel_tw.linear.x = 0
				self.vel_tw.angular.z = 0
				self.twistpub.publish(self.vel_tw)
				self.modepub.publish(self.mode)
				rate = rospy.Rate(0.2)
				rate.sleep()
			
			elif self.signl == 'TUNN':
				self.mode.mode = 4
				self.mode.cnt = 0
				#self.modepub.publish(self.mode)
			
			else:
				self.mode.mode = 0
				self.mode.cnt = 0
			self.modepub.publish(self.mode)
			#elif self.sign_numr != 0 :
				#print("ddd")
				#self.mode.mode = 0
				#self.mode.cnt =	1
				#self.vel_tw.linear.x =- 0.01
				#self.vel_tw.angular.z = 0
				#self.modepub.publish(self.mode)
				#self.twistpub.publish(self.vel_tw)
				#rate = rospy.Rate(0.2)
				#rate.sleep()
			#elif self.sign_numl != 0:
			#	print("ddfffd")
			#	self.mode.mode = 0
			#	self.mode.cnt =	1
			#	self.vel_tw.linear.x = -0.01
			#	self.vel_tw.angular.z = 0
				#self.modepub.publish(self.mode)
			#	#elf.twistpub.publish(self.vel_tw)
			#	rate = rospy.Rate(0.2)
			#	rate.sleep()	
			#i#f self.sign_numl2 != 0:
			#	self.mode.mode = 0
		#		self.mode.cnt =	2
		#		self.vel_tw.linear.x = 0.13138
		#		self.vel_tw.angular.z = -0.5
		#		self.twistpub.publish(self.vel_tw)
		#	elif self.sign_numr2 != 0:
		#		self.mode.mode = 0
		#		self.mode.cnt =	2
		#		self.vel_tw.linear.x = 0.13138
		#		self.vel_tw.angular.z = -0.5
		#		self.twistpub.publish(self.vel_tw)
			#mode=2,laser use
			
			#else:
				#self.mode.mode = 0
				#self.mode.cnt = 0
		#lane =False
		elif self.lane_yellow is False and self.lane_white is False:
			print("2")
			self.mode.mode = 5
			self.mode.cnt = 5
			if self.signl == 'TUNN':
				self.mode.mode = 3
				self.mode.cnt = 0
		print("mode:{}, cnt:{}, sr:{},snr{}, sl:{}, snl:{}".format(self.mode.mode,self.mode.cnt,self.signr,self.sign_numr,self.signl, self.sign_numl))
		#self.modepub.publish(self.mode)
		print("yello{}, wh{}".format(self.lane_yellow,self.lane_white))
		rospy.loginfo("%d" % (self.mode.mode))

	def lanemsg(self,a):
		self.lane_yellow = a.yellow
		self.lane_white= a.white

	def lasermsg(self,a):
		self.laser_bool = a.bool

	def detectmsg(self,a):
		self.bar = a.bar
	def trafficmsg(self,a):
		self.traffic_light = a.traffic_light

	def signmsgl(self,a):
		self.sign_numl2 = a.data
		self.sign_numl = a.data2
		self.signl     = a.name
	def signmsgr(self,a):
		self.sign_numr2 = a.data
		self.sign_numr = a.data2
		self.signr     = a.name
def main():
	rospy.init_node('mode_ch')
	te = test()
	while not rospy.is_shutdown():
		te.modech()
		rospy.sleep(0.1)	

        
        
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass

    

