import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
img = cv.imread('14.jpg')
gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
corners = cv.goodFeaturesToTrack(gray,200,0.01,30,3)
corners = np.int0(corners)
for i in corners:
    x,y = i.ravel()
    cv.circle(img,(x,y),6,255,-1)
plt.imshow(img),plt.show()
