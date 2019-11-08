#!/usr/bin/env python
import rospy
from newmode.msg import mode_msg, twist
from geometry_msgs.msg import Twist


class twist_trans():
	def __init__(self):
		self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		self.pub1 = Twist()
		self.tw_sub = rospy.Subscriber("/mode_twist", twist, self.callback)

	def callback(self,d):
		
		
		self.pub1.linear.x = d.linear.x
		self.pub1.angular.z = d.angular.z

		#print("1")
		self._pub.publish(self.pub1)


		
def main():
	rospy.init_node('twist_ch')
	mo=twist_trans()
	rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
