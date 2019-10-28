#!/usr/bin/env python
import rospy
from newmode.msg import msg_sign



class test_sub():
	def __init__(self):

		self.tw_sub = rospy.Subscriber('/sign_msg', msg_sign, self.callback)

	def callback(self,d):
		
		print(d.data,d.name)
		


		
def main():
	rospy.init_node('test_sub')
	test = test_sub()
	rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
