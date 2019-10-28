#!/usr/bin/env python 
import cv2
import rospy
import os
import numpy as np
from newmode.msg import msg_sign_r,msg_sign_l
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
orb = cv2.ORB_create()
bridge = CvBridge()

class TrafficSign:
	def __init__(self):
		self.sign_msg = msg_sign_r()
		self.signpub = rospy.Publisher('/sign_msg_r', msg_sign_r, queue_size=10)
		self.img_sub = rospy.Subscriber('/image_right', Image, self.img_msg_right)
		self.data_left = rospy.Subscriber('/sign_msg_l',msg_sign_l, self.imgleft)
		self.sign_left = None
	def imgleft(self,msg):
		self.sign_left = msg.name	

	def img_msg_right(self,msg):
		try:
			self.frame = bridge.imgmsg_to_cv2(msg,"bgr8")
		except CvBridgeError as e:
			print(e)

	def set(self):
		self.path = "/home/ri/soonrobo/src/newmode/sample/"
		self.noi = cv2.imread("{}noimg.jpeg".format(self.path), cv2.IMREAD_COLOR)
		self.no = cv2.resize(self.noi, dsize=(200, 200), interpolation=cv2.INTER_LINEAR)
		
		self.signtime = 2
		self.out = []
		self.result = [0,0]
		self.sign_turn = [0,1]
		self.num = 1

		self.SignColor = []
		self.Gray = []
		self.Resize = []
		self.Blur = []
		self.Canny = []
		self.Signkp = []
		self.Signdes = []
				
		#open to match trafficsigns
		self.SignColor.append(cv2.imread("{}avoiiiiiii.png".format(self.path), cv2.IMREAD_COLOR))
		self.SignColor.append(cv2.imread("{}PARK.jpg".format(self.path), cv2.IMREAD_COLOR))

		for sample in self.SignColor:
			if sample is None: 
				print('no image')
				return 0		

		for size in range(len(self.SignColor)):	
			#resize
			self.Resize.append(cv2.resize(self.SignColor[size], dsize=(200, 200), interpolation=cv2.INTER_LINEAR))
			#Grayscale
			self.Gray.append(cv2.cvtColor(self.Resize[size], cv2.COLOR_BGR2GRAY))
			#blur
			self.Blur.append(cv2.medianBlur(self.Gray[size], 5))
			#canny
			self.Canny.append(cv2.Canny(self.Blur[size], 50, 200))
			#orb feature save
			print(size)	
			kp, des  = orb.detectAndCompute(self.Canny[size], None)
			self.Signkp.append(kp)
			self.Signdes.append(des)

	#grayscale change	
	def gray(self, image):
		imgGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		return imgGray

	#blur median
	def medianblur(self, image):
		imgMedian = cv2.medianBlur(image, 5)
		return imgMedian

	#canny edge
	def cannyedge(self, image):
		imgCanny = cv2.Canny(image, 50, 200)
		return imgCanny

	#ORB
	def orb(self, image, imageC):
		self.imgkp, self.imgdes = orb.detectAndCompute(imageC, None)

	#FLANN
	def flann(self, image, factor):
		matching = None
		matches = []
		gmatch = [[],[],[],[]]
		self.out = []
		sign = 0
		FLANN_INDEX_LSH = 6 
		index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_levle=1)
		search_params = dict(checks=50) #if checks value up? slow
		flann = cv2.FlannBasedMatcher(index_params, search_params)

		try:
			for des in self.Signdes:
				matches.append(flann.knnMatch(des, self.imgdes, k=2))

			for match in matches: 					
				try:			
					for m, n in match:
						if m.distance < factor*n.distance:
							gmatch[sign].append(m)	
				except:
					print('no mathch')
				if len(gmatch[sign]) > 15:
					self.out.append(sign)
					matching = cv2.drawMatches(self.Resize[sign], self.Signkp[sign], image, self.imgkp, gmatch[sign], matching, flags=2)
				else:
					self.out.append('')	
					if matching is None:
						matching = cv2.drawMatches(self.no, self.Signkp[sign], image, self.imgkp, gmatch[sign], matching, flags=2)
				sign += 1
			cv2.imshow('match', matching)
			self.error = 0
		except:
			print('error pop')		 			
			self.error = 1 			


	def findsign(self):
		self.sign_msg.data =0
		while True:
			if '' in self.out:
				self.out.remove('')
			else:
				break
		if len(self.out) == 0:
			#print('no match!!!!')
			self.result = [0,0]
			self.num = 1

		elif len(self.out) == 1:
			sign = self.out[0]
			if self.result[sign] == 0: 
				self.result = [0,0]
				self.result[sign] = 1
				self.num = 2
			elif self.result[sign] >= 1:
				self.result[sign] = self.num
				self.num += 1
			print(self.result)	
			
		else:
			sign1 = self.out[0]
			sign2 = self.out[1]
			if self.result[sign1] > 0:
				self.num += 1
				self.result[sign1] = self.num	
			elif self.result[sign2] > 0:
				self.num += 1
				self.result[sign2] = self.num	
			else:
				self.result = [0,0]
				self.num = 1
			print(self.result)
			
			
		self.sign_msg.data2=self.result[0]
		#self.signpub.publish(self.sign_msg)
		if self.sign_left == 'LEFT':
			self.sign_turn = [1,0]
		if self.sign_left == 'RIGH':
			self.sign_turn = [1,0]

		elif self.sign_turn[0] == 1 and self.result[0] >= self.signtime:
			print('AVOI')
			self.sign_msg.name = 'AVOI'
			self.sign_msg.data = 1
			#self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,1]
			self.num = 1
		elif self.sign_turn[1] == 1 and self.result[1] >= self.signtime:
			print('PARK')
			self.sign_msg.name = 'PARK'
			self.sign_msg.data = 1
			#self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0]
			self.num = 1
		else:
			self.sign_msg.data =0
		self.signpub.publish(self.sign_msg)

		
def main():
	rospy.init_node('traff_r')
	t = TrafficSign()
	t.set()

	while 1:
		
		frame = t.frame
		cv2.imshow('rifht',frame)
		#grayscale
		frameG = t.gray(frame)	

		#choose one - blur
		frameM = t.medianblur(frameG)

		#canny	
		frameC = t.cannyedge(frameM)

		#orb
		t.orb(frame,frameC)
	
		#FLANN
		t.flann(frame, 0.7)
		if t.error == 0:
		#sign return
			t.findsign()
			t.signpub.publish(t.sign_msg)
		else:
			pass
		if cv2.waitKey(10) == ord('q'):
			cv2.destroyAllWindows()
			return 0
if __name__ == "__main__":

	try:
		main()
	except TypeError as te:
		print("typeError", te)
			#cv2.imshow("show", self.Resize[size])	
			#while True:
			#	if cv2.waitKey(33) == ord('q'):
			#		cv2.destroyAllWindows()
			#		break	





