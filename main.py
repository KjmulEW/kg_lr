from PIL import Image
import numpy as np
import math
import kg_algs

CENTER_X = 500
CENTER_Y = 500
CENTER_Z = 500


class image_obj:

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
                    self.verts.append(list(map(float, x.split()[1:])))
                    flag = True
                else:
                    if flag:
                        break
        return self

    def draw_vert(self, filename, k):
        global CENTER_X, CENTER_Y
        for item in self.verts:
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
                    self.poly.append(
                        list(map(int, [xsplit[1].split('/')[0], xsplit[2].split('/')[0], xsplit[3].split('/')[0]])))
                    flag = True
                else:
                    if flag:
                        break
        return self

    def draw_poly(self, filename, k):
        for item in self.poly:
            for i in range(3):
                kg_algs.br_alg(-(self.verts[item[i] - 1][1] * k + CENTER_X),
                               -(self.verts[item[i] - 1][0] * k + CENTER_Y),
                               -(self.verts[item[(i + 1) % 3] - 1][1] * k + CENTER_X),
                               -(self.verts[item[(i + 1) % 3] - 1][0] * k + CENTER_Y),
                               self.image_matrix_f)
        image = Image.fromarray(self.image_matrix_f, mode='RGB')
        image.save(filename)


model1 = image_obj()
model2 = image_obj()
model1.read_vert(open("model_1.obj", "r"))
model2.read_vert(open("model_2.obj", "r"))
model1.draw_vert('vert1.png', 5000)
model2.draw_vert('vert2.png', 1 / 3)
model1.read_poly(open("model_1.obj", "r"))
model2.read_poly(open("model_2.obj", "r"))
model1.draw_poly('poly1.png', 5000)
model2.draw_poly('poly2.png', 1 / 3)
