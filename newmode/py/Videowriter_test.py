#!/usr/bin/env python

import numpy as np
import cv2
		
def main():
	cap = cv2.VideoCapture(0)
	if not cap.isOpened():
        	print("open fail camera")
	while True:
		ret, frame = cap.read()
		angle = 0

		if ret:
			lsd = cv2.createLineSegmentDetector(0)
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			crop_L = hsv[150:420, :220]
        		crop_R = frame[150:420, 402:]
        		L_mask = cv2.inRange(crop_L,(20,50,100),(70,255,255))
        		R_mask = cv2.inRange(crop_R, (230, 230, 230), (255, 255, 255))
	        	yello_line = lsd.detect(L_mask)
	        	white_line = lsd.detect(R_mask)
	        	img = np.hstack([L_mask, R_mask])
	        	
	        	cv2.imshow("img", img)
	        	
	        	x0_ye = None
	        	y0_ye = None
	        	x1_ye = None
	        	y1_ye = None
	        	x0_wh = None
	        	y0_wh = None
	        	x1_wh = None
	        	y1_wh = None
	        	
	        	if yello_line[0] is not None:
	        		slope_max = 0
				for dline in yello_line[0]:
					x0 = int(round(dline[0][0]))
					y0 = int(round(dline[0][1]))
					x1 = int(round(dline[0][2]))
					y1 = int(round(dline[0][3]))
					
					#try:
					#	slope = (y1 - y0)/(x1 - x0)
					#except:
					#	pass
					if (x1 - x0) == 0:
						slope = 999
					else:
						slope = (y1 - y0)/(x1 - x0)
						
					if slope > slope_max:
						slope_max = slope
						x0_ye, y0_ye, x1_ye, y1_ye = x0, y0, x1, y1
						cv2.line(frame, (x0, y0), (x1, y1), (255, 0, 0), 2)
						
			if white_line[0] is not None:
				slope_min = 1000		
				for dline in white_line[0]:
					x0 = int(round(dline[0][0]))
					y0 = int(round(dline[0][1]))
					x1 = int(round(dline[0][2]))
					y1 = int(round(dline[0][3]))
					
					if (x1 - x0) == 0:
						slope = 999
					else:
						slope = (y1 - y0)/(x1 - x0)
				
					if slope < slope_min:
						slope_min = slope
						x0_wh, y0_wh, x1_wh, y1_wh = x0, y0, x1, y1
						cv2.line(frame, (x0, y0), (x1, y1), (255, 0, 0), 2)
	        
			if yello_line[0] is not None and white_line[0] is None:
	        		y = frame.shape[0]
				x = int(frame.shape[1]/2)
				if x2_ye is not None and y2_ye is not None:
					angle = np.arctan2((y0_ye -y1_ye), (x0_ye - x1_ye)) * (180/np.pi)
					cv2.line(frame, (x, y), (x2_ye, y2_ye), (0,0,255), 2)
				
			elif yello_line[0] is None and white_line[0] is not None:
				y = frame.shape[0]
				x = int(frame.shape[1]/2)
				if y1_wh is not None and x1_wh is not None:
					angle = np.arctan2((y0_wh -y1_wh), (x0_wh - x1_wh)) * (180/np.pi)
					cv2.line(frame, (x0_wh, y0_wh), (x1_wh, y1_wh), (0,0,255), 2)
				
			elif yello_line[0] is not None and white_line[0] is not None:
				y = frame.shape[0]
				x = int(frame.shape[1]/2)
				if (y1_ye and x1_ye) is not None:
					y_mid = int((y1_wh + y1_ye)/2)
					x_mid = int((x1_wh + x1_ye)/2)
					angle = np.arctan2((y - y_mid), (x -x_mid)) * (180/np.pi)
					cv2.line(frame, (x, y), (x_mid, y_mid), (0,0,255), 2)
				else:
					if (y0_wh is not None and y0_ye is not None):
						y_mid = int((y0_wh + y0_ye)/2)
						x_mid = int((x0_wh + x0_ye)/2)
						angle = np.arctan2((y - y_mid), (x -x_mid)) * (180/np.pi)
						cv2.line(frame, (x, y), (x_mid, y_mid), (0,0,255), 2)
						
			else:
				pass
				
			print(angle)
			cv2.imshow("a", frame)
			
			if cv2.waitKey(10) == ord('q'):
				cv2.destroyAllWindows()
			
			
			
main()
	  


