import cv2 as cv
import os

linux_user = os.getlogin()
img = cv.imread(f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/map/warehouse.png", cv.IMREAD_GRAYSCALE)
rotated = cv.rotate(img, cv.ROTATE_180)
cv.imwrite(f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/map/warehouse.jpg", rotated)
