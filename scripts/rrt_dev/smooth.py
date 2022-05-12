import numpy as np
import matplotlib.pyplot as plt
from Map import DEFAULT_MAP1


class SmoothPath:
    def __init__(self, init_point, path):
        if not isinstance(init_point, tuple):
            raise TypeError("init_point must be a tuple")
        if len(init_point) != 2:
            raise ValueError("init_point must be a tuple of size 2")
        if not isinstance(path, list):
            raise TypeError("path must be a list")

        self.init_point = init_point
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


if __name__ == "__main__":
    p = SmoothPath((1, 1), [(18, 13), (17, 17)])
    p.generate(100)
    print(p.path)
    x = [pos[0] for pos in p.path]
    y = [pos[1] for pos in p.path]
    plt.imshow(DEFAULT_MAP1)
    plt.plot(y, x, 'r-')

    x = [pos[0] for pos in p.raw_path]
    x.insert(0, p.init_point[0])
    y = [pos[1] for pos in p.raw_path]
    y.insert(0, p.init_point[1])
    plt.plot(y, x, 'b-')
    plt.show()
