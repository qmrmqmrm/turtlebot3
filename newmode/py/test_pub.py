#!/usr/bin/env python
import rospy
import sys, select, tty, termios
from geometry_msgs.msg import Twist
from newmode.msg import mode_msg , start, msg_detect, msg_lane, msg_sign_l



def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":

	settings = termios.tcgetattr(sys.stdin)
    
	rospy.init_node('test_pub')
	pub = rospy.Publisher('/lane_msg', msg_lane, queue_size=10)
	pub_det = rospy.Publisher('/det_msg',msg_detect, queue_size = 10)
	pub_start = rospy.Publisher('/start',start, queue_size =10)
	signpub = rospy.Publisher('/sign_msg_l', msg_sign_l, queue_size=10)
	lane = msg_lane()
	sign_msg_l = msg_sign_l()
	det=msg_detect()
	starting = start()
    

	try:
		while(1):
			key = getKey()
			if key == 'q':
				lane.yellow = True
				lane.white = True
				
				print("q")
			elif key == 'w':
				lane.yellow = False
				lane.white = False
				print("w")
			elif key == 'a':
				det.bar = True
				print("a")
			elif key == 's':
				det.bar = False
				print("s")
			elif key == 'z':
				det.traffic_light = True
				print("z")
			elif key == 'x':
				det.traffic_light = False
				print("x")     
			
			if key == '1':
				print('LEFT')      
				sign_msg_l.data = 1
				sign_msg_l.name = 'LEFT'
				#self.signpub.publish(self.sign_msg)
			elif key == '2':
				print('RIGH')     
				sign_msg_l.data = 2
				sign_msg_l.name = 'RIGH'
				#self.signpub.publish(self.sign_msg)
			elif key == '3':
				print('TUNNSIGN')      
				sign_msg_l.data = 3
				sign_msg_l.name = 'TUNN'
				#self.signpub.publish(self.sign_msg)
			
			elif key == '4':
				print('AVOID SIGN')
				sign_msg_l.data = 4
				sign_msg_l.name = 'AVOI'
				#self.signpub.publish(self.sign_msg)
			elif key == '5':
				print('PARK')      
				sign_msg_l.data = 5
				sign_msg_l.name = 'PARK'
			elif key == '6':
				print('DONT')
				sign_msg_l.data = 6
				sign_msg_l.name = 'DONT'
				#self.signpub.publish(self.sign_msg)
			elif key == '0':
				print('None')
				sign_msg.data = 0
			else :
				if (key == '\x03'):
					sign_msg.data = 0
					break

			pub.publish(lane)
			#pub_det.publish(det)
			pub_start.publish(starting)
			signpub.publish(sign_msg_l)

	except rospy.ROSInterruptException: pass
       
