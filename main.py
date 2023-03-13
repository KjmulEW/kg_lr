from PIL import Image
import numpy as np
import math
import kg_algs

class image_obj:


    CENTER_X=500
    CENTER_Y=500
    CENTER_Z=500


    def __init__(self):
        self.verts = []
        self.poly = []
        self.image_matrix_v = np.zeros((1000, 1000, 3), dtype=np.uint8)
        self.image_matrix_f = np.zeros((1000, 1000, 3), dtype=np.uint8)

    def read_vert(self, file):
        for x in file:
            if x.split():
                flag = False
                if x.split()[0] == "v":
                    self.v_arr.append(list(map(float, x.split()[1:])))
                    flag = True
                else:
                    if flag:
                        break
        return self

    def draw_vert(self, filename, k):
        global CENTER_X, CENTER_Y
        for item in self.v_arr:
            y = -int(item[0] * k) + CENTER_Y
            x = -int(item[1] * k) + CENTER_X
            self.image_matrix_v[x, y] = [255, 255, 255]
        image = Image.fromarray(self.image_matrix_v, mode='RGB')
        image.save(filename)

    def read_poly(self, file):
        for x in file:
            xsplit = x.split()
            if xsplit:
                flag = False
                if xsplit[0] == "f":
                    self.f_arr.append(
                        list(map(int, [xsplit[1].split('/')[0], xsplit[2].split('/')[0], xsplit[3].split('/')[0]])))
                    flag = True
                else:
                    if flag:
                        break
        return self

    def draw_poly(self, filename, k):
        for item in self.f_arr:
            for i in range(3):
                br_alg(-(self.v_arr[item[i] - 1][1] * k + CENTER_X), -(self.v_arr[item[i] - 1][0] * k + CENTER_Y),
                       -(self.v_arr[item[(i+1)%3] - 1][1] * k + CENTER_X), -(self.v_arr[item[(i+1)%3] - 1][0] * k + CENTER_Y),
                       self.image_matrix_f)
        image = Image.fromarray(self.image_matrix_f, mode='RGB')
        image.save(filename)

