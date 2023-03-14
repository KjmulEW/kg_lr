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
        self.normals = []
        self.image_matrix_v = np.zeros((1000, 1000, 3), dtype=np.uint8)
        self.image_matrix_f = np.zeros((1000, 1000, 3), dtype=np.uint8)
        self.image_matrix_t = np.zeros((1000, 1000, 3), dtype=np.uint8)

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
                    self.normals.append(kg_algs.triangle_normal(self.verts[self.poly[-1][0]-1],self.verts[self.poly[-1][1]-1], self.verts[self.poly[-1][2]-1] ))
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

    def draw_poly_new_cos(self, filename, k):
        j = 0
        for item in self.poly:
            tcos = kg_algs.triangle_cos(self.normals[j])
            j+=1
            if tcos < 0:
                for i in range(3):
                    kg_algs.br_alg(-(self.verts[item[i] - 1][1] * k + CENTER_X),
                                   -(self.verts[item[i] - 1][0] * k + CENTER_Y),
                                   -(self.verts[item[(i + 1) % 3] - 1][1] * k + CENTER_X),
                                   -(self.verts[item[(i + 1) % 3] - 1][0] * k + CENTER_Y),
                                   self.image_matrix_f)
        image = Image.fromarray(self.image_matrix_f, mode='RGB')
        image.save(filename)

    def draw_triangle(self, x0, y0, x1, y1, x2, y2, k):
        x0 = x0 * k + CENTER_X
        y0 = y0 * k + CENTER_Y
        x1 = x1 * k + CENTER_X
        y1 = y1 * k + CENTER_Y
        x2 = x2 * k + CENTER_X
        y2 = y2 * k + CENTER_Y
        xmin = int(min(x0, x1, x2))
        ymin = int(min(y0, y1, y2))
        xmax = int(max(x0, x1, x2))
        ymax = int(max(y0, y1, y2))
        if (xmin < 0): xmin = 0
        if (ymin < 0): ymin = 0
        if (xmax < 0): xmax = 1000
        if (ymax < 0): xmin = 1000
        rand_color = np.random.randint(256, size=3)
        for x in range(xmin, xmax + 1):
            for y in range(ymin, ymax + 1):
                bar_cord = kg_algs.bar_cord(x, y, x0, y0, x1, y1, x2, y2)
                if bar_cord[0] > 0 and bar_cord[1] > 0 and bar_cord[2] > 0:
                    self.image_matrix_t[x, y] = rand_color

    def draw_triangles(self, filename, k):
        for item in self.poly:
            self.draw_triangle(-self.verts[item[0] - 1][1], -self.verts[item[0] - 1][0], -self.verts[item[1] - 1][1],
                               -self.verts[item[1] - 1][0], -self.verts[item[2] - 1][1], -self.verts[item[2] - 1][0], k)
            self.save_triangle(filename)

    def save_triangle(self, filename):
        image = Image.fromarray(self.image_matrix_t, mode="RGB")
        image.save(filename)

model1 = image_obj()
model2 = image_obj()

model1.read_vert(open("model_1.obj", "r"))
model2.read_vert(open("model_2.obj", "r"))

model1.read_poly(open("model_1.obj", "r"))
model2.read_poly(open("model_2.obj", "r"))

model1.draw_poly_new_cos('poly1_12.png', 5000)
model2.draw_poly_new_cos('poly2_12.png', 1 / 3)


