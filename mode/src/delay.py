#!/usr/bin/env python
import rospy
import sys, os
#from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,Int32
from newmode.msg import msg_delay_send,msg_delay_pl
class Delay:
	def __init__(self):
		self.pub = rospy.Publisher('delay_send',msg_delay_send,queue_size=1)
		self.sub =rospy.Subscriber('delay_pl',msg_delay_pl,self.submsg)
		self.pubmsg = msg_delay_send()
		self.pubmsg.delay = False
		self.demode = 0
	def sleep(self,a):
		self.pubmsg.delay  = True
		self.pub.publish(self.pubmsg)
		b = a
		print(a)
		rate = rospy.Rate(b)
		print("b")
		rate.sleep()
		print("fin:")
		
		print("a")
		rate.sleep()
		self.pubmsg.delay  = False
		self.pub.publish(self.pubmsg)
		
		
		

	def submsg(self,msg):
		self.demode = msg.delay_p
		
def main():
	rospy.init_node("delay")
	de = Delay()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if de.demode != 0:
		#a=1
			de.sleep(de.demode)
		else :
			de.pubmsg.delay = False
			de.pub.publish(de.pubmsg)
		rate.sleep()
		       
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass

    

