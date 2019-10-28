#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from newmode.msg import mode_msg
import random
import std_srvs.srv

GOAL_POSE = [[-1.7049, 0.3300, -1.0,1.0],
			[-1.6549, -1.6999, 0.0,1.0],
			[0.3699, -1.7799, 0.0,1.0]]


class NavigationStressTest(object):
	def __init__(self):
		self.event_in = None
		self.nav_goal = None
        
		self.nav_pub = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped,queue_size =10)
		self.modesub = rospy.Subscriber('/mode_msg',mode_msg, self.modemsg)
		self.mode = None
		
	def modemsg(self,msg):
		self.mode = msg.mode

    
	def run(self):        
        #rospy.sleep(10)
        #clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)
        #rospy.sleep(10)
        ##self.publish_random_goal()
		while not rospy.is_shutdown():
        		if self.mode == 4:
				self.publish_middle_goal()
				rospy.sleep(10)
				self.publish_exit_goal()
				break
			else:
				pass

	def publish_middle_goal(self):
		pose = GOAL_POSE[1]
		self.nav_goal = geometry_msgs.msg.PoseStamped()
		self.nav_goal.header.frame_id = '/map'
		self.nav_goal.pose.position.x = pose[0]
		self.nav_goal.pose.position.y = pose[1]
		self.nav_goal.pose.orientation.z = pose[2]
		self.nav_goal.pose.orientation.w = pose[3]
		self.nav_pub.publish(self.nav_goal)		

	def publish_exit_goal(self):
		pose = GOAL_POSE[2]
		self.nav_goal = geometry_msgs.msg.PoseStamped()
		self.nav_goal.header.frame_id = '/map'
		self.nav_goal.pose.position.x = pose[0]
		self.nav_goal.pose.position.y = pose[1]
		self.nav_goal.pose.orientation.z = pose[2]
		self.nav_goal.pose.orientation.w = pose[3]
		self.nav_pub.publish(self.nav_goal)	

def main():
	rospy.init_node("nav_goal")
	nav_stress_test = NavigationStressTest()
	nav_stress_test.run()

if __name__ == '__main__':
	main()





