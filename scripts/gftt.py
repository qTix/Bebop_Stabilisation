import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import os

datasource ="../raw_images/"
flights = [x[0] for x in os.walk(datasource)]
dataset = flights[1]
datapoints = os.listdir(os.path.join(datasource,dataset))
datapoints.sort()

def get_points_from_img(img):
	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	corners = cv.goodFeaturesToTrack(gray,20,0.50,20,3)
	corners = np.int0(corners)
	return corners


for i, datapoint in enumerate(datapoints):
	img = cv.imread(os.path.join(datasource,dataset,datapoint))
	img_next = cv.imread(os.path.join(datasource,dataset,datapoints[i+1]))
	prev_pts = get_points_from_img(img)
	curr_pts = get_points_from_img(img_next)
	for i in prev_pts:
    		x,y = i.ravel()
    		cv.circle(img,(x,y),6,255,-1)
	print("previous points : ",len(prev_pts))
	for i in curr_pts:
		x,y = i.ravel()
		cv.circle(img_next,(x,y),6,255,-1)
	print("current points : ",len(curr_pts))
	f = plt.figure()
	f.add_subplot(1,2,1)
	plt.imshow(img)
	f.add_subplot(1,2,2)
	plt.imshow(img_next)
	plt.show()
	m = cv.estimateAffine2D(prev_pts[:4], curr_pts[:4], None)
	print(m)
