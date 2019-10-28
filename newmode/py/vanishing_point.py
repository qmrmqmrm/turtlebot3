#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from newmode.msg import msg_lane
from newmode.msg import msg_sign


class Image_class:
    def __init__(self):
        self.bridge = CvBridge()
        self.angle_msg = msg_lane()
        self.line = []
        self.line_detect = False
        self.right_lane_mode = False
        self.left_lane_mode = False
        self.lane_mode = False
        self.angle = 0
        self.vanishing_x = 0
        self.vanishing_y = 0
        self.pub_lane = rospy.Publisher('/lane_msg', msg_lane, queue_size=1)
        self.sub_sign = rospy.Subscriber('/sign_msg',msg_sign, self.signmsg)

    def signmsg(self,msg):
        self.sign = msg.name

    def frame_img(self, image):
        self.ori_img = image
        self.img_rst = self.ori_img
        self.img_copy = self.img_rst

    def wh_mask(self):
        img_rgb = self.img_rst
        img_hls = self.img_rst
        # change_hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        lower_rgb_wh = np.array([220, 220, 220])
        upper_rgb_wh = np.array([255, 255, 255])
        on_v = cv2.inRange(img_rgb, lower_rgb_wh, upper_rgb_wh)
        img_rgb = cv2.bitwise_and(img_rgb, img_rgb, mask=on_v)

        change_hls = cv2.cvtColor(img_hls, cv2.COLOR_BGR2HLS)
        lower_hls_wh = np.array([0, 220, 0])
        upper_hls_wh = np.array([179, 255, 255])
        on_v = cv2.inRange(change_hls, lower_hls_wh, upper_hls_wh)
        img_hls = cv2.bitwise_and(img_hls, img_hls, mask=on_v)

        self.img_wh_mask = cv2.bitwise_and(img_hls, img_rgb)
        self.img_wh_mask = img_hls

    def ye_mask(self):
        img_hsv = self.img_rst
        img_hls = self.img_rst
        img_rgb = self.img_rst
        
        lower_rgb_ye = np.array([130, 190, 190])
        upper_rgb_ye = np.array([255, 255, 255])
        on_v = cv2.inRange(img_rgb, lower_rgb_ye, upper_rgb_ye)
        img_rgb = cv2.bitwise_and(img_rgb, img_rgb, mask=on_v)

        change_hls = cv2.cvtColor(img_hls, cv2.COLOR_BGR2HLS)
        lower_rgb_ye = np.array([20, 0, 0])
        upper_rgb_ye = np.array([70, 255, 255])
        on_v = cv2.inRange(change_hls, lower_rgb_ye, upper_rgb_ye)
        img_hls = cv2.bitwise_and(img_hls, img_hls, mask=on_v)

        change_hsv = cv2.cvtColor(img_hsv, cv2.COLOR_BGR2HSV)
        lower_ye = np.array([20, 90, 50])
        upper_ye = np.array([70, 255, 255])
        on_v = cv2.inRange(change_hsv, lower_ye, upper_ye)
        img_hsv = cv2.bitwise_and(img_hsv, img_hsv, mask=on_v)
        self.img_ye_mask = cv2.bitwise_and(img_hsv, img_hls)
        self.img_ye_mask = img_rgb

    def edge(self):
        img = cv2.bitwise_or(self.img_wh_mask, self.img_ye_mask)
        #img = self.img_wh_mask
        #img = self.img_rst
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.img_mask = img
        blur = cv2.GaussianBlur(img, (3, 3), 2)
        what, blur = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        self.img_canny = cv2.Canny(blur, (self.img_copy.shape[1] + self.img_copy.shape[0]) / 4, (self.img_copy.shape[1] + self.img_copy.shape[0]) / 2)

    def get_line(self):
        self.line_r = cv2.HoughLines(self.img_canny, rho=1, theta=np.pi / 180, threshold=65, srn=0, stn=0, min_theta=np.pi / 2, max_theta=(3 * np.pi) / 4)
        self.line_l = cv2.HoughLines(self.img_canny, rho=1, theta=np.pi / 180, threshold=65, srn=0, stn=0, min_theta=0, max_theta=3*np.pi / 8)
		#elif len(self.line) is None:
		#	pass
        if self.line_l is None and self.line_r is None:
            self.line_detect = False
        else:
            self.line_detect = True

    def theta_min(self):
        theta_min = np.pi / 2
        rho_min = np.pi
        if self.line_l is not None:
        	for line in self.line_l:
            		for rho, theta in line:
                		if theta < theta_min:
                    			theta_min = theta
                    			rho_min = rho

        self.theta_l = theta_min
       # print("l", self.rho_min)
        self.rho_l = rho_min

    def theta_max(self):
        theta_max = 0
        rho_max = 0
        if self.line_r is not None:
        	for line in self.line_r:
            		for rho, theta in line:
                		if theta > theta_max:
                    			theta_max = theta
                    			rho_max = rho

        self.theta_r = theta_max
        self.rho_r = rho_max

    def determin_point_r(self):
        a = np.cos(self.theta_r)
        b = np.sin(self.theta_r)
        x0 = a * self.rho_r
        y0 = b * self.rho_r
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        # cv2.line(self.ori_img, (x1+180, y1), (x2+180, y2), (255, 0, 0), 1)
        self.point_r = [x1, y1, x2, y2]

        cv2.line(self.img_copy, (x1, y1), (x2, y2), (0, 0, 255), 2)
	#cv2.circle(self.img_copy, (x2 + 150, y2 + 100), 1, (0, 255, 255), 5)
        if self.point_r is not None:
            self.pre_point_r = self.point_r
            self.right_lane_mode = True
        else:
            self.right_lane_mode = False

    def determin_point_l(self):
        a = np.cos(self.theta_l)
        b = np.sin(self.theta_l)
        x0 = a * self.rho_l
        y0 = b * self.rho_l
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        #x1 = int(x0 + 1000 * (-b))
        #y1 = int(y0 + 1000 * a)
        #x2 = int(x0 - 1000 * (-b))
        #y2 = int(y0 - 1000 * a)
        # cv2.line(self.ori_img, (x1-180, y1), (x2-180, y2), (0, 255, 0), 1)
        self.point_l = [x1, y1, x2, y2]
        #print(self.point_l1, self.point_l2)

        cv2.line(self.img_copy, (x1, y1), (x2, y2), (255, 0, 0), 2)
        

        if self.point_l is not None:
            self.pre_point_l = self.point_l
            self.left_lane_mode = True

        else:
            self.left_lane_mode = False

    def vanishing_point(self, point_l, point_r):
        under = (point_l[3] - point_l[1]) * (point_r[2] - point_r[0]) - (point_l[2] - point_l[0]) * (point_r[3] - point_r[1])
            
        if under != 0 :
            
            _t = (point_l[2] - point_l[0]) * (point_r[1] - point_l[1]) - (point_l[3] - point_l[1]) * (point_r[0] - point_l[0])
            _s = (point_r[2] - point_r[0]) * (point_r[1] - point_l[1]) - (point_r[3] - point_r[1]) * (point_r[0] - point_l[0])
            
            t = _t/under
                
            if (_t == 0) and (_s == 0) :
                self.vanishing_x = int(point_r[0] + t * (point_r[2] + point_r[0]))
                self.vanishing_y = int(point_r[1] + t * (point_r[3] + point_r[1]))
                    
                cv2.line(self.img_copy, (int(self.img_copy.shape[1] / 2), self.img_copy.shape[0]), (self.vanishing_x, self.vanishing_y2),  (0, 255, 0), 2)
   	        		#cv2.line(self.img_copy, (point_r2[0] + 150, point_r2[1] + 100), (self.vanishing_x + 150, self.vanishing_y + 100),  (0, 255, 0), 2)
  	        		#print(self.vanishing_x, self.vanishing_y)
       
        

    def determine_course(self):
        if self.right_lane_mode and self.left_lane_mode:
            self.vanishing_point(self.point_l, self.point_r)
            y = self.img_mask.shape[0]
            x = int(self.img_mask.shape[1]/2)
            self.angle = np.arctan2((self.vanishing_y - y), (self.vanishing_x - x)) * (180/np.pi)

        elif self.right_lane_mode is True and self.left_lane_mode is False:
            self.vanishing_point(self.pre_point_l, self.pre_point_r)
            y = self.img_mask.shape[0]
            x = int(self.img_mask.shape[1]/2)
            self.angle = np.arctan2((self.vanishing_y - y), (self.vanishing_x - x)) * (180 / np.pi)

        elif self.right_lane_mode is False and self.left_lane_mode is True:
            self.vanishing_point(self.point_l, self.pre_point_r)
            y = self.img_mask.shape[0]
            x = int(self.img_mask.shape[1] / 2)
            self.angle = np.arctan2((self.vanishing_y - y), (self.vanishing_x - x)) * (180 / np.pi)


    def angle_pub(self):
        self.angle_msg.angle = self.angle
        self.pub_lane.publish(self.angle_msg)
        rospy.loginfo("%f" % (self.angle_msg.angle))
        self.__init__()

def main():
    pub = rospy.Publisher('/image_raw', Image, queue_size=1)
    #pub_lane = rospy.Publisher('/lane_msg', msg_lane, queue_size=1)
    rospy.init_node("lane")
    rate = rospy.Rate(10)
    img_now = Image_class()
    cap = cv2.VideoCapture(1)
    cap.set(3, 320)
    cap.set(4, 240)
    if not cap.isOpened():
        rospy.loginfo("open fail video")
    while not rospy.is_shutdown():
        ret, frame1 = cap.read()
        frame2 = frame1[50:, 0:]
        #frame = cv2.copyMakeBorder(frame1, 300, 0, 0, 0, cv2.BORDER_CONSTANT)

        if ret:
            img_now.frame_img(frame1)
            img_now.wh_mask()
            img_now.ye_mask()
            img_now.edge()
            img_now.get_line()
            if img_now.line_detect is True:
                img_now.theta_max()
                img_now.theta_min()
                img_now.determin_point_r()
                img_now.determin_point_l()
                img_now.lane_mode = img_now.right_lane_mode or img_now.left_lane_mode
                if img_now.lane_mode:
                    img_now.determine_course()
                    img_now.angle_pub()

            #frame_ = img_now.bridge.cv2_to_imgmsg(img_now.img_copy, "bgr8")
            #pub.publish(frame_)
            #pub_lane.publish(img_now.angle)
            cv2.imshow("1", img_now.img_mask)
            cv2.imshow("2", img_now.img_copy)

            k = cv2.waitKey(10)
            if k == 27:
                cap.release()
                cv2.destroyAllWindows()
                break


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

