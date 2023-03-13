import numpy as np
from PIL import Image
import numpy as np
import math


def br_alg(x0, y0, x1, y1, image_matrix):
    steep = False
    if abs(x0 - x1) < abs(y0 - y1):
        x0, y0 = y0, x0
        x1, y1 = y1, x1
        steep = True
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
    dx = int(x1 - x0)
    dy = int(y1 - y0)
    if dx == 0: dx = 0.0001
    derror = abs(dy / dx)
    error = 0
    y = int(y0)
    x = int(x0)
    while x <= x1:
        if steep:
            image_matrix[y, x] = [255, 255, 255]
        else:
            image_matrix[x, y] = [255, 255, 255]
        error += derror
        if error > 0.5:
            if y1 > y0:
                y += 1
            else:
                y -= 1
            error -= 1
        x += 1


def bar_cord(x, y, x0, y0, x1, y1, x2, y2):
    lambda0 = ((x1 - x2) * (y - y2) - (y1 - y2) * (x - x2)) / ((x1 - x2) * (y0 - y2) - (y1 - y2) * (x0 - x2))
    lambda1 = ((x2 - x0) * (y - y0) - (y2 - y0) * (x - x0)) / ((x2 - x0) * (y1 - y0) - (y2 - y0) * (x1 - x0))
    lambda2 = ((x0 - x1) * (y - y1) - (y0 - y1) * (x - x1)) / ((x0 - x1) * (y2 - y1) - (y0 - y1) * (x2 - x1))
    if sum(lambda0, lambda1, lambda2) != 1:
        print('Все плохо')
