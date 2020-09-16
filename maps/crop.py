import cv2 
import os
# Converts binary image to 3 channel image for drawing in color

files = os.listdir(".")
for file_name in files:
    if file_name[-4:] == ".png":
        image = file_name
        img = cv2.imread(image,cv2.IMREAD_COLOR)
        cv2.imwrite(image, img[20:-20,20:-20])
