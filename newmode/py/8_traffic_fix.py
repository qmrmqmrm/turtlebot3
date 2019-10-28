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
		self.frame_r = None
		self.frame_l = None
		self.signpub = rospy.Publisher('/sign_msg', msg_sign, queue_size=10)
		self.img_subr = rospy.Subscriber('/image_right', Image,self.img_msg_right)
		self.img_subl = rospy.Subscriber('/image_left', Image,self.img_msg_left)

	def img_msg_right(self,msg):
		try:
			self.frame_r = bridge.imgmsg_to_cv2(msg,"bgr8")
		except CvBridgeError as e:
			print(e)

	def img_msg_left(self,msg):
		try:
			self.frame_l = bridge.imgmsg_to_cv2(msg,"bgr8")
		except CvBridgeError as e:
			print(e)

		
	def set(self):
		self.path = "/home/ri/soonrobo/src/newmode/sample/"
		self.noi = cv2.imread("{}noimg.jpeg".format(self.path), cv2.IMREAD_COLOR)
		self.no = cv2.resize(self.noi, dsize=(200, 200), interpolation=cv2.INTER_LINEAR)
		
		self.signtime = 5
		self.out = []
		self.result_r = [0,0,0,0,0,0]
		self.result_l = [0,0,0,0,0,0]
		self.sign_turn = [0,0,1,0,0,0]
		self.num = 1
		self.camera_rl = [0,0]

		self.SignColor = []
		self.Gray = []
		self.Resize = []
		self.Blur = []
		self.Canny = []
		self.Signkp = []
		self.Signdes = []
				
		#open to match trafficsigns
		self.SignColor.append(cv2.imread("{}LEFT.jpg".format(self.path), cv2.IMREAD_COLOR)) #l
		self.SignColor.append(cv2.imread("{}RIGH.jpg".format(self.path), cv2.IMREAD_COLOR)) #r
		self.SignColor.append(cv2.imread("{}DONT.jpg".format(self.path), cv2.IMREAD_COLOR)) #r l
		self.SignColor.append(cv2.imread("{}AVOI.jpg".format(self.path), cv2.IMREAD_COLOR)) #r
		self.SignColor.append(cv2.imread("{}PARK.jpg".format(self.path), cv2.IMREAD_COLOR)) #r
		self.SignColor.append(cv2.imread("{}TUNN.jpg".format(self.path), cv2.IMREAD_COLOR)) #l

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
	def orb(self, image_r, imageC_r,image_l, imageC_l):
		self.imgkp_r, self.imgdes_r = orb.detectAndCompute(imageC_r, None)
		self.imgkp_l, self.imgdes_l = orb.detectAndCompute(imageC_l, None)

	#FLANN
	def flann(self, image_r, image_l, factor):
		matching_r = None
		matching_l = None
		matches_r = []
		matches_l = []
		gmatch_r = [[],[],[],[],[],[],[],[]]
		gmatch_l = [[],[],[],[],[],[],[],[]]
		out_r = []
		out_l = []
		sign = 0
		FLANN_INDEX_LSH = 6 
		index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_levle=1)
		search_params = dict(checks=50) #if checks value up? slow
		flann = cv2.FlannBasedMatcher(index_params, search_params)

		#try:
		for des in self.Signdes:
			matches_r.append(flann.knnMatch(des, self.imgdes_r, k=2))
			matches_l.append(flann.knnMatch(des, self.imgdes_l, k=2))

		for match in matches_r: 					
			try:			
				for m, n in match:
					if m.distance < factor*n.distance:
						gmatch_r[sign].append(m)	
			except:
				print('no mathch')
			if len(gmatch_r[sign]) > 20:
				out_r.append(sign)
				matching_r = cv2.drawMatches(self.Resize[sign], self.Signkp[sign], image_r, self.imgkp_r, gmatch_r[sign], matching_r, flags=2)
			else:
				out_r.append('')	
				if matching_r is None:
					matching_r = cv2.drawMatches(self.no, self.Signkp[sign], image_r, self.imgkp_r, gmatch_r[sign], matching_r, flags=2)
			sign += 1
		sign = 0
		for match in matches_l: 					
			try:			
				for m, n in match:
					if m.distance < factor*n.distance:
						gmatch_l[sign].append(m)	
			except:
				print('no mathch')
			if len(gmatch_l[sign]) > 20:
				out_l.append(sign)
				matching_l = cv2.drawMatches(self.Resize[sign], self.Signkp[sign], image_l, self.imgkp_l, gmatch_l[sign], matching_l, flags=2)
			else:
				out_l.append('')	
				if matching_l is None:
					matching_l = cv2.drawMatches(self.no, self.Signkp[sign], image_l, self.imgkp_l, gmatch_l[sign], matching_l, flags=2)
			sign += 1
		#print(self.out)
		cv2.imshow('match_r', matching_r)
		cv2.imshow('match_l', matching_l)
		return out_r,out_l
		#except:
		#	print('error pop')		 			


	def findsign(self, out, result):
		while True:
			if '' in out:
				out.remove('')
			else:
				break
		
		if len(out) == 0:
			print('no match!!!!')
			result = [0,0,0,0,0,0]
			self.num = 1

		elif len(out) == 1:
			sign = out[0]
			if result[sign] == 0: 
				result = [0,0,0,0,0,0]
				result[sign] = 1
				self.num = 2
			elif result[sign] >= 1:
				result[sign] = self.num
				self.num += 1
			print(result)	
			
		elif len(out) == 2:
			sign1 = out[0]
			sign2 = out[1]
			if result[sign1] > 0:
				self.num += 1
				result[sign1] = self.num	
			elif result[sign2] > 0:
				self.num += 1
				result[sign2] = self.num	
			else:
				result = [0,0,0,0,0,0]
				self.num = 1
			print(result)	

		else:
			print(len(out))
		return result

	def give_sign(self):
		if self.sign_turn[0] == 1 and self.result_l[0] >= self.signtime:
			print('LEFT')
			self.sign_msg.name = 'LEFT'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,1,0,0,0]
			self.num = 1
		elif self.sign_turn[1] == 1 and self.result_r[1] >= self.signtime:
			print('RIGH')
			self.sign_msg.name = 'RIGH'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,1,0,0,0]
			self.num = 1
		elif self.sign_turn[2] == 1 and (self.result_r[2] >= self.signtime or self.result_l[2] >= self.signtime):
			print('DONT')
			self.sign_msg.name = 'DONT'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,0,1,0,0]
			self.num = 1
		elif self.sign_turn[3] == 1 and self.result_r[3] >= self.signtime:
			print('AVOI')
			self.sign_msg.name = 'AVOI'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,0,0,1,0]
			self.num = 1
		elif self.sign_turn[4] == 1 and self.result_r[4] >= self.signtime:
			print('PARK')
			self.sign_msg.name = 'PARK'
			self.sign_msg.data = 1
			self.signpub.publish(self.sign_msg)
			self.sign_turn = [0,0,0,0,0,1]
			self.num = 1
		elif self.sign_turn[5] == 1 and self.result_l[5] >= self.signtime:
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
			
		#frame_r = cv2.resize(t.frame_r, dsize=(640, 320), interpolation=cv2.INTER_LINEAR)
		#frame_l = cv2.resize(t.frame_l, dsize=(640, 320), interpolation=cv2.INTER_LINEAR)
		frame_r = t.frame_r
		frame_l = t.frame_l		
		#cv2.imshow('frame', frame_r)
		
		#grayscale
		frameG_r = t.gray(frame_r)
		frameG_l = t.gray(frame_l)			

		#choose one - blur
		frameM_r = t.medianblur(frameG_r)
		frameM_l = t.medianblur(frameG_l)
		#canny	
		frameC_r = t.cannyedge(frameM_r)
		frameC_l = t.cannyedge(frameM_l)

		#orb
		t.orb(frame_r,frameC_r,frame_l,frameC_l)
	
		#FLANN
		out_r, out_l = t.flann(frame_r,frame_l, 0.7)
		
		#sign return
		t.result_r = t.findsign(out_r, t.result_r)
		t.resulr_l = t.findsign(out_l, t.result_l)
		t.give_sign()
		
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





