import rospy
import sys, os
from commonmsgs.msg import msg_mode, msg_lane, msg_detect, msg_laser, msg_sign_l, msg_sign_r, twist, msg_traffic

class mode():
    def __init__(self):
        
        self.lane_yellow = False
        self.lane_white = False
        self.laser_bool = False

        self.signr = None
        self.signl = None
        #subscriber
        
        self.lanesub = rospy.Subscriber('/lane_msg', msg_lane, self.lanemsg)
        self.lasersub = rospy.Subscriber('/msg_laser',msg_laser, self.lasermsg)
        self.signsubl = rospy.Subscriber('/sign_msg_l', msg_sign_l, self.signmsgl)
        self.signsubr = rospy.Subscriber('/sign_msg_r', msg_sign_r, self.signmsgr)
        #publshing
        
        self.mode = mode_msg()
        self.vel_tw = twist()
        self.modepub = rospy.Publisher('/mode_msg',msg_mode,queue_size=10)
        self.twistpub = rospy.Publisher('/mode_twist', twist, queue_size=10)
        self.sign_numr= None
        self.sign_numl= None
        self.sign_numr2= None
        self.sign_numl2= None
        
    def modechange(self):
        if self.lane_yellow is True or self.lane_white is True:
            if self.laser_bool == True
                self.mode_mode = 2
                self.mode.cnt = 0
            
            else:
                self.mode.mode = 0
                self.mode.cnt = 0
            self.modepub.publish(self.mode)
        elif self.lane_yellow is False and self.lane_white is False:
            self.mode.mode =5
            self.mode.cnt = 5
        print("mode:{}, cnt:{}, sr:{},snr{}, sl:{}, snl:{}".format(self.mode.mode,self.mode.cnt,self.signr,self.sign_numr,self.signl, self.sign_numl))
		#self.modepub.publish(self.mode)
		print("yello{}, wh{}".format(self.lane_yellow,self.lane_white))
		rospy.loginfo("%d" % (self.mode.mode))
            
    def lanemsg(self,a):
		self.lane_yellow = a.yellow
		self.lane_white= a.white
    def lasermsg(self,a):
		self.laser_bool = a.bool
		
		
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
            
            
            
            
            
