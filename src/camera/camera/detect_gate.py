# To detect green side of gate. There is a chance that there are black tapes on the gate, so we want to outline the entire pole of the gate
# to do so, we find the two corners of the pole and draw a final rectangle around it


import cv2
import numpy as np
from matplotlib import pyplot as plt

image = cv2.imread('rulebook_gate.png')
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower_green = np.array([40, 100, 100])
upper_green = np.array([75, 255, 255])

#This code detects the green side of the gave
mask_green = cv2.inRange(hsv, lower_green, upper_green)
contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#Top left coords
topmost_y = float('inf')  
topmost_x = float('inf')

#Bottom right coords
bottommost_y = 0 
bottommost_x = 0

for contour in contours_green:
    x, y, w, h = cv2.boundingRect(contour)
    topmost_y = min(topmost_y, y)
    topmost_x = min(topmost_x, x)
    bottommost_y = max(bottommost_y, y + h)
    bottommost_x = max(bottommost_x, x + w)

#Draws a rectangle bounded by the top left point and bottom right point 
cv2.rectangle(image, (topmost_x, topmost_y), (bottommost_x, bottommost_y), (255, 0, 0), 2)
height = bottommost_y - topmost_y # Height to calculate the ratio of left to right side

plt_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
plt.imshow(image)
plt.axis('off')