#!/usr/bin/env python
"""
Created on Fri Nov  9 10:25:22 2018
@author: mason
"""
'''libraries'''
import time
import numpy as np
import rospy
import roslib
import cv2
from sensor_msgs.msg import Image

from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError

global LSD
global cap_center
LSD = cv2.createLineSegmentDetector(0)
cap_center = cv2.VideoCapture(1)
bridge = CvBridge()
''' class '''
class robot():
    def __init__(self):
        #rospy.init_node('robot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def keeping(self,hsv):
        global LSD
        vel_msg=Twist()
        crop_L=hsv[350:410,40:220]
        crop_R=hsv[350:410,402:582]
        L_mask = cv2.inRange(crop_L,(21,50,100),(36,255,255))
        L2_mask = cv2.inRange(crop_L,(36,0,165),(255,255,255))
        R_mask = cv2.inRange(crop_R,(36,0,165),(255,255,255))
        R2_mask = cv2.inRange(crop_R,(21,50,100),(36,255,255))
        yello_line = LSD.detect(L_mask)
        yello_line2 = LSD.detect(L2_mask)
        white_line = LSD.detect(R_mask)
        white_line2 = LSD.detect(R2_mask)
        
        if yello_line[0] is None and yello_line2[0] is None:
            vel_msg.linear.x = 0.05
            vel_msg.angular.z = 0.6
        elif white_line[0] is None and white_line2[0] is None:
            vel_msg.linear.x = 0.05
            vel_msg.angular.z = -0.6
        else :
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def imageupdate(self):
        global cap_center
        ret, image=cap_center.read()
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        return image,hsv
rospy.init_node('robot_controller', anonymous=True)       
turtle=robot()
time.sleep(1.2)
if __name__=='__main__':
    while not rospy.is_shutdown():
        
        pub_center = rospy.Publisher('/image_raw_center', Image, queue_size=1)
        ret_center, frame_center = cap_center.read()
        frame_center_ = bridge.cv2_to_imgmsg(frame_center, "bgr8")
        pub_center.publish(frame_center_)
        try:
            img,hsv=turtle.imageupdate()
            turtle.keeping(hsv)
            cv2.imshow("l", img[350:410,40:220])
        except :
           print('got error')
