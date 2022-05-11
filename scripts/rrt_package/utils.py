import numpy as np


def plot_path_coord(path):
    y = list(map(lambda y: y.getPos("y"), path))
    x = list(map(lambda x: x.getPos("x"), path))
    return y, x


def gen_point(width, height):
    return np.random.randint(width), np.random.randint(height)


def nearest(list_node, pos):
    return min(list_node, key=lambda node: node.distance_from(pos))


def checkpoints(x1, y1, x2, y2):
    if abs(x1 - x2) > abs(y1 - y2):
        to_check_x = range(min(x1, x2) + 1, max(x1, x2))
        a = (y1 - y2) / (x1 - x2)
        b = y1 - a * x1
        to_check_y = [round(a * x_i + b) for x_i in to_check_x]
    elif abs(x1 - x2) < abs(y1 - y2):
        to_check_y = range(min(y1, y2) + 1, max(y1, y2))
        a = (x1 - x2) / (y1 - y2)
        b = x1 - a * y1
        to_check_x = [round(a * y_i + b) for y_i in to_check_y]
    else:  # means that (x1,y1) = (x2,y2)
        to_check_x = to_check_y = None
    return to_check_x, to_check_y




