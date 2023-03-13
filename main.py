from PIL import Image
import numpy as np
import math
import kg_algs

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
                    self.v_arr.append(list(map(float, x.split()[1:])))
                    flag = True
                else:
                    if flag:
                        break
        return self