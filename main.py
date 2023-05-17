from PIL import Image
import numpy as np
import math
import kg_algs

CENTER_X = 700
CENTER_Y = 500
CENTER_Z = 500
d = 1000


class image_obj:

    def __init__(self):
        self.verts = []
        self.poly = []
        self.normals = []
        self.image_matrix_v = np.zeros((1000, 1000, 3), dtype=np.uint8)
        self.image_matrix_f = np.zeros((1000, 1000, 3), dtype=np.uint8)
        self.image_matrix_t = np.zeros((1000, 1000, 3), dtype=np.uint8)
        self.z_buff = np.random.randint(0,100, (1000,1000))

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

    def rotate_x(self, angle):
        angle = math.radians(angle)
        for i in range(len(self.verts)):
            y = self.verts[i][1]
            z = self.verts[i][2]
            self.verts[i][1] = y * math.cos(angle) - z * math.sin(angle)
            self.verts[i][2] = z * math.cos(angle) + y * math.sin(angle)

    def rotate_z(self, angle):
        angle = math.radians(angle)
        for i in range(len(self.verts)):
            x = self.verts[i][0]
            y = self.verts[i][1]
            self.verts[i][0] = x * math.cos(angle) - y * math.sin(angle)
            self.verts[i][1] = y * math.cos(angle) + x * math.sin(angle)
    def rotate_y(self, angle):
        angle = math.radians(angle)
        for i in range(len(self.verts)):
            x = self.verts[i][0]
            z = self.verts[i][2]
            self.verts[i][0] = x * math.cos(angle) - z * math.sin(angle)
            self.verts[i][2] = z * math.cos(angle) + x * math.sin(angle)

    def read_normals(self, file):
        for x in file:
            if x.split():
                flag = False
                if x.split()[0] == "vn":
                    self.normals.append(list(map(float, x.split()[1:])))
                    flag = True
                else:
                    if flag:
                        break
    def projective_transform(self, x, y, z, d):
        xp = d * x / (z + d)
        yp = d * y / (z + d)
        return [xp, yp]
    def draw_vert(self, filename, k):
        global CENTER_X, CENTER_Y
        for item in self.verts:
            x, y = self.projective_transform(-item[1], -item[0], -item[2], d)
            x = int(x * k) + CENTER_X
            y = int(y * k) + CENTER_Y
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
                    self.normals.append(
                        kg_algs.triangle_normal(self.verts[self.poly[-1][0] - 1], self.verts[self.poly[-1][1] - 1],
                                                self.verts[self.poly[-1][2] - 1]))
                    flag = True
                else:
                    if flag:
                        break

    def draw_poly(self, filename, k):
        for item in self.poly:
            for i in range(3):
                x1, y1 = self.projective_transform(-self.verts[item[i] - 1][1], -self.verts[item[i] - 1][0], -self.verts[item[i] - 1][2], d)
                x2, y2 = self.projective_transform(-self.verts[item[(i + 1) % 3] - 1][1], -self.verts[item[(i + 1) % 3] - 1][0], -self.verts[item[(i + 1) % 3] - 1][2], d)
                kg_algs.br_alg(x1 * k + CENTER_X,
                               y1 * k + CENTER_Y,
                               x2 * k + CENTER_X,
                               y2 * k + CENTER_Y,
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
                    x1, y1 = self.projective_transform(-self.verts[item[i] - 1][1], -self.verts[item[i] - 1][0], -self.verts[item[i] - 1][2], d)
                    x2, y2 = self.projective_transform(-self.verts[item[(i + 1) % 3] - 1][1], -self.verts[item[(i + 1) % 3] - 1][0], -self.verts[item[(i + 1) % 3] - 1][2], d)
                    kg_algs.br_alg(x1 * k + CENTER_X,
                                   y1 * k + CENTER_Y,
                                   x2 * k + CENTER_X,
                                   y2 * k + CENTER_Y,
                                   self.image_matrix_f)
        image = Image.fromarray(self.image_matrix_f, mode='RGB')
        image.save(filename)

    def draw_triangle(self, x0, y0, z0, x1, y1, z1, x2, y2, z2, k, tcos):
        x0, y0 = self.projective_transform(x0, y0, z0, d)
        x1, y1 = self.projective_transform(x1, y1, z1, d)
        x2, y2 = self.projective_transform(x2, y2, z2, d)
        x0 = int(x0 * k) + CENTER_X
        y0 = int(y0 * k) + CENTER_Y
        x1 = int(x1 * k) + CENTER_X
        y1 = int(y1 * k) + CENTER_Y
        x2 = int(x2 * k) + CENTER_X
        y2 = int(y2 * k) + CENTER_Y
        xmin = int(min(x0, x1, x2))
        ymin = int(min(y0, y1, y2))
        xmax = int(max(x0, x1, x2))
        ymax = int(max(y0, y1, y2))
        if (xmin < 0): xmin = 0
        if (ymin < 0): ymin = 0
        if (xmax < 0): xmax = 1000
        if (ymax < 0): xmin = 1000

        shade_color = [255 * tcos, 255 * tcos, 255 * tcos]
        for x in range(xmin, xmax + 1):
            for y in range(ymin, ymax + 1):
                bar_cord = kg_algs.bar_cord(x - CENTER_X / k , y - CENTER_Y / k , x0 - CENTER_X / k , y0 - CENTER_Y / k , x1 - CENTER_X / k , y1 - CENTER_Y / k , x2 - CENTER_X / k , y2 - CENTER_Y / k )
                if bar_cord[0] > 0 and bar_cord[1] > 0 and bar_cord[2] > 0:
                    z_poly = z0*bar_cord[0] + z1*bar_cord[1] + z2*bar_cord[2]
                    if z_poly < self.z_buff[x,y]:
                        self.image_matrix_t[x,y] = shade_color
                        self.z_buff[x,y] = z_poly

    def save_triangle(self, filename):
        image = Image.fromarray(self.image_matrix_t, mode="RGB")
        image.save(filename)
    def draw_triangles(self,filename,k):
        j=0
        for item in self.poly:
            x0=-self.verts[item[0]-1][1]
            y0=-self.verts[item[0]-1][0]
            z0=-self.verts[item[0]-1][2]
            x1=-self.verts[item[1]-1][1]
            y1 = -self.verts[item[1] - 1][0]
            z1 = -self.verts[item[1] - 1][2]
            x2 = -self.verts[item[2] - 1][1]
            y2 = -self.verts[item[2] - 1][0]
            z2 = -self.verts[item[2] - 1][2]
            tcos = kg_algs.triangle_cos(self.normals[j])
            j += 1
            if tcos > 0:
                self.draw_triangle(x0, y0, z0, x1, y1, z1, x2, y2, z2, k, tcos)
                self.save_triangle(filename)
    def gouraud_shading(self, light):
        vertex_colors = []
        for normal in self.normals:
            l0 = np.dot(normal[0],light) / np.linalg.norm(normal[0]) / np.linalg.norm(light)
            l1 = np.dot(normal[1], light) / np.linalg.norm(normal[1]) / np.linalg.norm(light)
            l2 = np.dot(normal[2], light) / np.linalg.norm(normal[2]) / np.linalg.norm(light)

            intensity = np.dot(normal, light)
            color = [intensity * 255, intensity * 255, intensity * 255]
            vertex_colors.append(color)
        return vertex_colors



model1 = image_obj()
model2 = image_obj()

model1.read_vert(open("model_1.obj", "r"))
model2.read_vert(open("model_2.obj", "r"))

model2.rotate_z(30)  #16 задание. Добавлены 3 метода, можно вращать модели по координатам.

model1.read_poly(open("model_1.obj", "r"))
model2.read_poly(open("model_2.obj", "r"))

#15 задание. Меняем координаты z у всех моделей. Так же добавлена ф-ция projective_transform
z_offset = abs(min([vert[2] for vert in model1.verts])) + 1
for vert in model1.verts:
    vert[2] += z_offset

z_offset = abs(min([vert[2] for vert in model2.verts])) + 1
for vert in model2.verts:
    vert[2] += z_offset


model2.draw_triangles("triangle2_44.png", 1 / 3)
#model1.draw_triangles("triangle2_34.png", 5000)