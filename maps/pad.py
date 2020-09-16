import cv2
import os
# Converts binary image to 3 channel image for drawing in color

files = os.listdir(".")
for file_name in files:
    if file_name[-4:] == ".png":
        image = file_name
        img = cv2.imread(image,cv2.IMREAD_COLOR)
	img = cv2.copyMakeBorder(img, 20,20,20,20, cv2.BORDER_CONSTANT, (0,0,0))
        cv2.imwrite(image, img)
