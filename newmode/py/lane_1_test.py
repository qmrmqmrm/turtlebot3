#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from newmode.msg import msg_lane
from newmode.msg import msg_sign

class Image_class:
    def __init__(self):
        self.bridge = CvBridge()
        self.angle_msg = msg_lane()
        self.line = []
        self.line_md = []
        self.ye_pose = []
        self.wh_pose = []
        self.lane_mode = False
        self.ye_true = False
        self.wh_true = False
        self.x1_ye = None
        self.x2_ye = None
        self.y1_ye = None
        self.y2_ye = None
        self.x1_wh = None
        self.x2_wh = None
        self.y1_wh = None
        self.y2_wh = None
        self.sign = None
        self.angle = None
        self.pub_lane = rospy.Publisher('/lane_msg', msg_lane, queue_size=1)
        self.sub_sign = rospy.Subscriber('/sign_msg',msg_sign, self.signmsg)

    def signmsg(self,msg):
        self.sign = msg.name

    def frame_img(self, image):
        self.ori_img = image
        self.img_rst = self.ori_img[50:180, 10:310]
        self.img_copy = self.img_rst.copy()


    def wh_mask(self):
        img_rgb = self.img_rst
        img_hls = self.img_rst
        lower_rgb_wh = np.array([245, 245, 245])
        upper_rgb_wh = np.array([255, 255, 255])
        on_v = cv2.inRange(img_rgb, lower_rgb_wh, upper_rgb_wh)
        img_rgb = cv2.bitwise_and(img_rgb, img_rgb, mask=on_v)

        change_hls = cv2.cvtColor(img_hls, cv2.COLOR_BGR2HLS)
        lower_hls_wh = np.array([0, 240, 0])
        upper_hls_wh = np.array([179, 255, 255])
        on_v = cv2.inRange(change_hls, lower_hls_wh, upper_hls_wh)
        img_hls = cv2.bitwise_and(img_hls, img_hls, mask=on_v)

        self.img_wh_mask = cv2.bitwise_and(img_hls, img_rgb)
        self.img_wh_mask = img_rgb


    def ye_mask(self):
        img_hsv = self.img_rst
        img_hls = self.img_rst

        change_hls = cv2.cvtColor(img_hls,  cv2.COLOR_BGR2HLS)
        lower_rgb_ye = np.array([20, 50, 150])
        upper_rgb_ye = np.array([70, 255, 255])
        on_v = cv2.inRange(change_hls, lower_rgb_ye, upper_rgb_ye)
        img_hls = cv2.bitwise_and(img_hls, img_hls, mask=on_v)

        change_hsv = cv2.cvtColor(img_hsv, cv2.COLOR_BGR2HSV)
        lower_ye = np.array([20, 90, 50])
        upper_ye = np.array([70, 255, 255])
        on_v = cv2.inRange(change_hsv, lower_ye, upper_ye)
        img_hsv = cv2.bitwise_and(img_hsv, img_hsv, mask=on_v)
        self.img_ye_mask = cv2.bitwise_and(img_hsv, img_hls)
        self.img_ye_mask = img_hls


    def edge(self):
        img = cv2.bitwise_or(self.img_wh_mask, self.img_ye_mask)
        self.img_mask = img
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(img, (3, 3), 3)
        what, blur = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        self.img_mask = blur
        self.img_canny = cv2.Canny(blur, 150, 200)

    def get_line(self):
        self.line = cv2.HoughLinesP(self.img_canny, 1, np.pi / 180, 40, minLineLength=70,maxLineGap=15)

        if self.line is None:
            self.line_detect = False

        elif self.line is not None:
            self.line_detect = True


    def average_slope_intercept(self):
        slope_max_wh = -1
        slope_max_ye = -1

        for line in self.line:
            for x1, y1, x2, y2 in line:
                if x2 == x1:
                    continue

                slope = (y2 - y1) / (x2 - x1)
                slope_abs = abs(slope)

                if slope < 0:
                    if slope_max_ye < slope_abs:
                        slope_max_ye = slope_abs
                        self.ye_pose = [x1, y1, x2, y2]
                        #cv2.line(self.img_copy, (x1_ye, y1_ye), (x2_ye, y2_ye), [255, 0, 0], 2)

                elif slope >= 0:
                    if slope_max_wh < slope_abs:
                        slope_max_wh = slope_abs
                        self.wh_pose = [x1, y1, x2, y2]


        if not self.ye_pose == []:
            self.ye_true = True
            #self.angle.wh = self.ye_true
            self.x1_ye, self.y1_ye, self.x2_ye, self.y2_ye = self.ye_pose
            self.angle_msg.x1_ye = self.x1_ye
            self.angle_msg.y1_ye = self.y1_ye
            self.angle_msg.x2_ye = self.x2_ye
            self.angle_msg.y2_ye = self.y2_ye
            cv2.line(self.img_copy, (self.x1_ye, self.y1_ye), (self.x2_ye, self.y2_ye), [255, 0, 0], 2)

        if not self.wh_pose == []:
            self.wh_true = True
            #self.angle.ye = self.ye_true
            self.x1_wh, self.y1_wh, self.x2_wh, self.y2_wh = self.wh_pose
            self.angle_msg.x1_ye = self.x1_wh
            self.angle_msg.y1_ye = self.y1_wh
            self.angle_msg.x2_ye = self.x2_wh
            self.angle_msg.y2_ye = self.y2_wh
            cv2.line(self.img_copy, (self.x1_wh, self.y1_wh), (self.x2_wh, self.y2_wh), [0, 0, 255], 2)

        if self.wh_true or self.ye_true:
            self.lane_mode = True
        else:
            self.lane_mode = False

        self.angle_msg.sw = self.lane_mode


    def go_straight(self):
        x1 = int((self.x1_wh + self.x2_ye) // 2)
        x2 = int((self.x2_wh + self.x1_ye) // 2)
        y1 = int((self.y1_wh + self.y2_ye) // 2)
        y2 = int((self.y2_wh + self.y1_ye) // 2)

        if x2 == x1:
            pass

        else:
            self.angle = np.arctan2((y1 - y2), (x1 - x2)) * (180 / np.pi)

        cv2.line(self.img_copy, (x1, y1), (x2, y2), [0, 255, 0], 2)


    def go_left_curve(self):
            if self.x1_wh == self.x2_wh:
                pass
            else:
                self.angle = np.arctan2((self.y1_wh - self.y2_wh), (self.x1_wh - self.x2_wh)) * (180 / np.pi)

            cv2.line(self.img_copy, (self.x1_wh + 150, self.y1_wh), (self.x2_wh + 150, self.y2_wh), [0, 255, 0], 2)

    def go_right_curve(self):
        if self.x1_ye == self.x2_ye:
            pass
        else:
            self.angle = np.arctan2((self.y1_ye - self.y2_ye), (self.x1_ye - self.x2_ye)) * (180 / np.pi)

        cv2.line(self.img_copy, (self.x1_ye - 150, self.y1_ye), (self.x2_ye - 150, self.y2_ye), [0, 255, 0], 2)

    def determine_course(self, count_r, count_l):
        if self.wh_true and self.ye_true:
            self.go_straight()
            count_l = 0
            count_r = 0

        elif self.wh_true is False and self.ye_true is True:
            count_r = count_r + 1
            if count_r >= 5:
                self.go_right_curve()

        elif self.ye_true is False and self.wh_true is True:
            count_l = count_l + 1
            if count_l >= 5:
                self.go_left_curve()

        return count_r, count_l

    def angle_pub(self):
        if self.sign != "RIGHT SIGN" and self.sign != "LEFT SIGN" and self.sign != "CANTGO SIGN":
            if self.angle <= 180 and self.angle >= -180:
                if self.angle > -100 and self.angle < -85:
                    self.angle = -90
 
                else :
                    pass

            self.angle_msg.angle = self.angle        
            self.pub_lane.publish(self.angle_msg)
            #rospy.loginfo("%f" %(self.angle_msg.angle))
        
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
    count_r = 0
    count_l = 0
    if not cap.isOpened():
        rospy.loginfo("open fail video")
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        #frame = cv2.flip(frame, 0)
        #frame = cv2.flip(frame, 1)

        if ret:
            img_now.frame_img(frame)
            img_now.wh_mask()
            img_now.ye_mask()
            img_now.edge()
            img_now.get_line()
            if img_now.line_detect:
                img_now.average_slope_intercept()
                count_r, count_l = img_now.determine_course(count_r, count_l)
                if img_now.angle is not None:
                    img_now.angle_pub()

            frame_ = img_now.bridge.cv2_to_imgmsg(img_now.img_copy, "bgr8")
            pub.publish(frame_)
            #pub_lane.publish(img_now.angle)

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

