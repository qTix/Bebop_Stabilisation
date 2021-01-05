import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('17.jpg',0)
edges = cv2.Canny(img,100,200)
ret, thresh1 = cv2.threshold(edges,127,255,cv2.THRESH_BINARY)

plt.subplot(121),plt.imshow(edges,cmap = 'gray')
plt.title('edges Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(thresh1,cmap = 'gray')
plt.title('Threshed Image'), plt.xticks([]), plt.yticks([])

plt.show()
