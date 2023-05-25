import numpy as np
from PIL import Image
import numpy as np
import math
#la


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

    if ((x1 - x2) * (y0 - y2) - (y1 - y2) * (x0 - x2)):
        lambda0 = ((x1 - x2) * (y - y2) - (y1 - y2) * (x - x2)) / ((x1 - x2) * (y0 - y2) - (y1 - y2) * (x0 - x2))
    else:
        lambda0 = 0
    if (((x2 - x0) * (y1 - y0) - (y2 - y0) * (x1 - x0))):
        lambda1 = ((x2 - x0) * (y - y0) - (y2 - y0) * (x - x0)) / ((x2 - x0) * (y1 - y0) - (y2 - y0) * (x1 - x0))
    else:
        lambda1 = 0
    if (((x0 - x1) * (y2 - y1) - (y0 - y1) * (x2 - x1))):
        lambda2 = ((x0 - x1) * (y - y1) - (y0 - y1) * (x - x1)) / ((x0 - x1) * (y2 - y1) - (y0 - y1) * (x2 - x1))
    else:
        lambda2 = 0
    return [lambda0, lambda1, lambda2]


def triangle_normal(vert0, vert1, vert2):
    """
    Calculate the normal vector to a triangle given by three vertices with coordinates x, y, z.
    """
    # Calculate two vectors within the triangle
    v1 = np.array([vert1[0] - vert0[0], vert1[1] - vert0[1], vert1[2] - vert0[2]])
    v2 = np.array([vert2[0] - vert0[0], vert2[1] - vert0[1], vert2[2] - vert0[2]])

    # Calculate the cross product of the two vectors to get the normal vector
    normal = np.cross(v1, v2)

    # Normalize the normal vector
    normal = normal / np.linalg.norm(normal)

    return normal


def triangle_cos(n, l =[0,0,1]):
    t_cos = np.dot(n,l) / (np.linalg.norm(n)* np.linalg.norm(n))
    return t_cos


def HSBtoRGB(hue, saturation, brightness):
    c = brightness * saturation
    x = c * (1 - abs((hue / 60) % 2 - 1))
    m = brightness - c
    if 0 <= hue < 60:
        r, g, b = c, x, 0
    elif 60 <= hue < 120:
        r, g, b = x, c, 0
    elif 120 <= hue < 180:
        r, g, b = 0, c, x
    elif 180 <= hue < 240:
        r, g, b = 0, x, c
    elif 240 <= hue < 300:
        r, g, b = x, 0, c
    else:
        r, g, b = c, 0, x
    r = int((r + m) * 255)
    g = int((g + m) * 255)
    b = int((b + m) * 255)
    return [r,g,b]
