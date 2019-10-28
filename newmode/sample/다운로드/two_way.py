#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from newmode.msg import msg_lane, msg_sign

def main():
	msg = msg_lane()
	sign = msg_sign()
	angle = None
	
        lanepub=rospy.Publisher('/lane_msg', msg, queue_size=1)

	if sign.sign == "RIGHT SIGN":
		x1_ye = msg.x1_ye
		y1_ye = msg.y1_ye
		x2_ye = msg.x2_ye
		y2_ye = msg.y2_ye
		angle = np.arctan2((y1 - y2) , (x1 - x2)) * (180/np.pi)

	if sign.sign == "LEFT SIGN":
		x1_wh = msg.x1_wh
		y1_wh = msg.y1_wh
		x2_wh = msg.x2_wh
		y2_wh = msg.y2_wh
		angle = np.arctan2((y1 - y2) , (x1 - x2)) * (180/np.pi)

	if sign.sign == "DONT GO":
		if sign_1 == "RIGHT SIGN":
		    angle = np.arctan2((y1 - y2) , (x1 - x2)) * (180/np.pi)
		    msg.angle = angle

		if sign_1 == "LEFT SIGN":
		    angle = np.arctan2((y1 - y2) , (x1 - x2)) * (180/np.pi)

    if angle <= 180 and angle >= -180:
        if angle > 70 and angle < 110:
            angle = 90 - angle

        else :
            angle = angle / 2

    msg.angle = angle        
    rospy.loginfo("%f" %(angle))
    lanepub.publish(msg)	    

if sign.sign is not None:
	sign_1 = sign.sign



