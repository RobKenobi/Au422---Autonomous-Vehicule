import numpy as np
import matplotlib.pyplot as plt
from Map import DEFAULT_MAP1


class SmoothPathBezier:
    def __init__(self, starting_point, path):
        if not isinstance(starting_point, tuple):
            raise TypeError("init_point must be a tuple")
        if len(starting_point) != 2:
            raise ValueError("init_point must be a tuple of size 2")
        if not isinstance(path, list):
            raise TypeError("path must be a list")

        self.init_point = starting_point
        if path[0] == starting_point:
            p = path
            p.pop(0)
            self.raw_path = p
        self.raw_path = path
        self.path = list()

    def generate(self, number_points=100):
        t = np.linspace(0, 1, number_points)
        for t_i in t:
            points = self.raw_path.copy()
            points.insert(0, self.init_point)
            while len(points) > 1:
                first = points.pop(0)
                new = list()
                for i, point in enumerate(points):
                    x, y = point
                    x_new = (1 - t_i) * first[0] + t_i * x
                    y_new = (1 - t_i) * first[1] + t_i * y
                    new.append((x_new, y_new))
                    first = points[i]
                points = new
            self.path.append(points[0])
        self.path.pop(0)

    def getPath(self, init=False, smooth=True):
        if smooth:
            if len(self.path):
                if init:
                    p = self.path.copy()
                    p.insert(0, self.init_point)
                    return p
                else:
                    return self.path
            raise AttributeError("path hasn't been generated yet. Call generate() on SmoothPath object")
        else:
            if init:
                p = self.raw_path.copy()
                p.insert(0, self.init_point)
                return p
            else:
                return self.raw_path


if __name__ == "__main__":
    p = SmoothPathBezier((1, 1), [(18, 13), (17, 17)])
    p.generate(100)
    print(p.path)
    x = [pos[0] for pos in p.getPath(init=True, smooth=True)]
    y = [pos[1] for pos in p.getPath(init=True, smooth=True)]
    plt.imshow(DEFAULT_MAP1)
    plt.plot(y, x, 'r-')

    x = [pos[0] for pos in p.getPath(init=True, smooth=False)]
    y = [pos[1] for pos in p.getPath(init=True, smooth=False)]
    plt.plot(y, x, 'b-')
    plt.show()
