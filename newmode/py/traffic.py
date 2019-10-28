#!/usr/bin/env python 
import cv2
import rospy
import os
import numpy as np
from newmode.msg import msg_sign

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
orb = cv2.ORB_create()
bridge = CvBridge()
class TrafficSign:
	def __init__(self):
		self.POriginal = []
		self.sign_msg = msg_sign()
		self.frame = None	
		self.signpub = rospy.Publisher('/sign_msg', msg_sign, queue_size=10)
		self.img_sub = rospy.Subscriber('/image_raw_right', Image,self.img_msg)
	def img_msg(self,msg):
		try:
			self.frame = bridge.imgmsg_to_cv2(msg,"bgr8")
		except CvBridgeError as e:
			print(e)

		
	def set(self):
		self.path = "/home/ri/soonrobo/src/newmode/sample/"
		self.noi = cv2.imread("{}noimg.jpeg".format(self.path), cv2.IMREAD_COLOR)
		self.no = cv2.resize(self.noi, dsize=(200, 200), interpolation=cv2.INTER_LINEAR)
		
		self.signtime = 3
		self.out = []
		self.result = [0,0,0,0,0,0]
		self.sign_turn = [1,1,0,0,0,0]
		self.num = 1

		self.SignColor = []
		self.Gray = []
		self.Resize = []
		self.Blur = []
		self.Canny = []
		self.Signkp = []
		self.Signdes = []
				
		#open to match trafficsigns
		self.SignColor.append(cv2.imread("{}LEFT.jpg".format(self.path), cv2.IMREAD_COLOR))
		self.SignColor.append(cv2.imread("{}TWOW.jpg".format(self.path), cv2.IMREAD_COLOR))
		self.SignColor.append(cv2.imread("{}DONT.jpg".format(self.path), cv2.IMREAD_COLOR))
		self.SignColor.append(cv2.imread("{}AVOI.jpg".format(self.path), cv2.IMREAD_COLOR))
		self.SignColor.append(cv2.imread("{}PARK.jpg".format(self.path), cv2.IMREAD_COLOR))
		self.SignColor.append(cv2.imread("{}TUNN.jpg".format(self.path), cv2.IMREAD_COLOR))

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
		gmatch = [[],[],[],[],[],[],[],[]]
		self.out = []
		sign = 0
		FLANN_INDEX_LSH = 6 
		index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_levle=1)
		search_params = dict(checks=50) #if checks value up? slow
		flann = cv2.FlannBasedMatcher(index_params, search_params)
		#try:
		for des in self.Signdes:
			matches.append(flann.knnMatch(des, self.imgdes, k=2))

		for match in matches: 					
			try:			
				for m, n in match:
					if m.distance < factor*n.distance:
						gmatch[sign].append(m)	
			except:
				print('no mathch')
			if len(gmatch[sign]) > 20:
				self.out.append(sign)
				matching = cv2.drawMatches(self.Resize[sign], self.Signkp[sign], image, self.imgkp, gmatch[sign], matching, flags=2)
			else:
				self.out.append('')	
				if matching is None:
					matching = cv2.drawMatches(self.no, self.Signkp[sign], image, self.imgkp, gmatch[sign], matching, flags=2)
			sign += 1
		#print(self.out)
		cv2.imshow('match', matching)
		#except:
		#	print('error pop')		 			


	def findsign(self):
		while True:
			if '' in self.out:
				self.out.remove('')
			else:
				break
		if len(self.out) == 0:
			print('no match!!!!')
			self.result = [0,0,0,0,0,0]
			self.num = 1

		elif len(self.out) == 1:
			sign = self.out[0]
			if self.result[sign] == 0: 
				self.result = [0,0,0,0,0,0]
				self.result[sign] = 1
				self.num = 2
			elif self.result[sign] >= 1:
				self.result[sign] = self.num
				self.num += 1
			print(self.result)	
			
		elif len(self.out) == 2:
			sign1 = self.out[0]
			sign2 = self.out[1]
			if self.result[sign1] > 0:
				self.num += 1
				self.result[sign1] = self.num	
			elif self.result[sign2] > 0:
				self.num += 1
				self.result[sign2] = self.num	
			else:
				self.result = [0,0,0,0,0,0]
				self.num = 1
			print(self.result)	

		else:
			print(len(self.out))

		if self.sign_turn[0] == 1 and self.result[0] >= self.signtime:
			print('LEFT')
			self.sign_msg.name = 'LEFT'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,1,0,0,0]
			self.num = 1
		elif self.sign_turn[1] == 1 and self.result[1] >= self.signtime:
			print('RIGH')
			self.sign_msg.name = 'RIGH'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,1,0,0,0]
			self.num = 1
		elif self.sign_turn[2] == 1 and self.result[2] >= self.signtime:
			print('DONT')
			self.sign_msg.name = 'DONT'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,0,1,0,0]
			self.num = 1
		elif self.sign_turn[3] == 1 and self.result[3] >= self.signtime:
			print('AVOI')
			self.sign_msg.name = 'AVOI'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,0,0,1,0]
			self.num = 1
		elif self.sign_turn[4] == 1 and self.result[4] >= self.signtime:
			print('PARK')
			self.sign_msg.name = 'PARK'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,0,0,0,1]
			self.num = 1
		elif self.sign_turn[5] == 1 and self.result[5] >= self.signtime:
			print('TUNN')
			self.sign_msg.name = 'TUNN'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,0,0,0,0]
			self.num = 1
			
		
def main():
	rospy.init_node('traff_fix')
	t = TrafficSign()
	t.set()
	
	while 1:
			
		frame = cv2.resize(t.frame, dsize=(640, 320), interpolation=cv2.INTER_LINEAR)
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

		#sign return
		t.findsign()
		t.signpub.publish(t.sign_msg)

		if cv2.waitKey(10) == ord('q'):
			cv2.destroyAllWindows()
			return 0
	cap.release()
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





