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
		self.count = 0
		self.signr = None
		self.signl = None
		#subscriber
		#self.startsub = rospy.Subscriber('/start',start,self.starting)
		self.lanesub = rospy.Subscriber('/lane_msg', msg_lane, self.lanemsg)
		self.lasersub = rospy.Subscriber('/msg_laser',msg_laser, self.lasermsg)
		#self.signsubl = rospy.Subscriber('/sign_msg_l', msg_sign_l, self.signmsgl)
		self.signsubr = rospy.Subscriber('/sign_msg_l', msg_sign_l, self.signmsgl)
		
		#publshing
		self.mode = mode_msg()
		
		self.modepub = rospy.Publisher('/mode_msg',mode_msg,queue_size=10)
		self.sign_numr= None
		self.sign_numl= None
		self.sign_numr2= None
		self.sign_numl2= None
	
	def modech(self):
		#print("mode,lane",self.mode.mode,self.lane)
		if self.lane_yellow is True or self.lane_white is True:
		
			#print(self.signl)	
	 		if self.laser_bool == True:
	 			self.mode.mode = 2
	 			self.mode.cnt = 0
 			elif self.signl == "PARK" :
 				
				self.mode.mode = 3
				self.mode.cnt =0
			elif self.signl == 'PARK2':
				self.mode.mode = 0
				self.modepub.publish(self.mode)
				rate = rospy.Rate(1)
 				rate.sleep()
 				self.mode.mode = 3
				self.modepub.publish(self.mode)
			
				
		 	else:	
				self.mode.mode = 0
				self.mode.cnt = 0
			
	 				
		#lane =False
		elif self.lane_yellow is False and self.lane_white is False:
			self.mode.mode = 5
			self.mode.cnt = 5
		self.modepub.publish(self.mode)
			
		print("mode:{}, cnt:{}, ".format(self.mode.mode,self.mode.cnt))
		#self.modepub.publish(self.mode)
		print("yello:{}, wh:{}".format(self.signl,self.lane_white))
		rospy.loginfo("%d" % (self.mode.mode))

	def lanemsg(self,a):
		self.lane_yellow = a.yellow
		self.lane_white  = a.white

	def lasermsg(self,a):
		self.laser_bool = a.bool
	#def signmsgl(self,a):
		#self.sign_numr = a.data
		#self.signr     = a.name
	def signmsgl(self,a):
		self.sign_numl = a.data
		self.signl    = a.name
	
	
def main():
	rospy.init_node('control')
	te = test()
	while not rospy.is_shutdown():
		te.modech()
		rospy.sleep(0.1)	

        
        
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass

    

