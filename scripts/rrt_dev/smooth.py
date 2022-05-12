import numpy as np
import matplotlib.pyplot as plt
from Map import DEFAULT_MAP1

original_path = [(1, 1), (12, 7), (17, 17)]


def new_list(points, t):
    first = points.pop(0)
    new = list()
    for i, point in enumerate(points):
        x, y = point
        x_new = (1 - t) * first[0] + t * x
        y_new = (1 - t) * first[1] + t * y
        new.append((x_new, y_new))
        first = points[i]
    return new


def smooth_path_bezier(path, number_points, plot=False):
    t = np.linspace(0, 1, number_points)

    if not isinstance(path, list):
        raise TypeError("path must be a list")

    if plot:
        x = [pos[0] for pos in path]
        y = [pos[1] for pos in path]
        plt.plot(y, x, '-y')

    path = list()
    for t_i in t:
        points = path.copy()
        print(len(points))
        while len(points) >1:
            points = new_list(points, t_i)
        path.append(points[0])

    if plot:
        x = [pos[0] for pos in path]
        y = [pos[1] for pos in path]
        plt.plot(y, x, 'r-')
        plt.imshow(DEFAULT_MAP1)
        plt.show()
    return path


smooth1 = smooth_path_bezier(original_path, 100, True)

