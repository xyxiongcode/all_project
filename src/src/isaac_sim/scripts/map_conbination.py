import cv2 as cv
import numpy as np
import os

rows = 2
columns = 2
resolution = 0.05
interval = 1

linux_user = os.getlogin()
single_map = cv.imread(f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/map/isaac_map.png", cv.IMREAD_GRAYSCALE)
(height, width) = single_map.shape

multi_map_height = int(height * rows + (rows - 1) * (interval / resolution))
multi_map_width = int(width * columns + (columns - 1) * (interval / resolution))
multi_map = np.full(shape=(multi_map_height, multi_map_width), fill_value=255, dtype=np.uint8)

for i in range(rows):
    for j in range(columns):
        row_start = int(i * height + i * (interval / resolution))
        column_start = int(j * width + j * (interval / resolution))
        multi_map[row_start:row_start + height, column_start:column_start + width] = single_map

cv.imwrite(f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/map/multi_map.pgm", multi_map)
