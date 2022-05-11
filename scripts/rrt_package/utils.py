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
        to_check_x = range(min(x1, x2) + 1, max(x1, x2) + 1)
        a = (y1 - y2) / (x1 - x2)
        b = y1 - a * x1
        to_check_y = [round(a * x_i + b) for x_i in to_check_x]
    elif abs(x1 - x2) < abs(y1 - y2):
        to_check_y = range(min(y1, y2) + 1, max(y1, y2) + 1)
        a = (x1 - x2) / (y1 - y2)
        b = x1 - a * y1
        to_check_x = [round(a * y_i + b) for y_i in to_check_y]
    else:  # means that (x1,y1) = (x2,y2)
        to_check_x = to_check_y = None
    return to_check_x, to_check_y


def saturation(value, maximum, minimum=0):
    return min(max(minimum, value), maximum)


def square(env, x, y, square_size, max_x, max_y):
    return env[saturation(x - square_size // 2, max_x):saturation(x + square_size // 2, max_x)][
           saturation(y - square_size // 2, max_y):saturation(y + square_size // 2, max_y)]


def check_line(env, pos1, pos2, robot_size=2, obstacle=1, debug=False):
    to_check_x, to_check_y = checkpoints(pos1[0], pos1[1], pos2[0], pos2[1])

    if to_check_x is None:
        if debug:
            print("checkpoints returned None")
        return False

    width, height = env.shape

    for x, y in zip(to_check_x, to_check_y):
        if np.sum(square(env, x, y, robot_size, width - 1, height - 1) == obstacle) != 0:
            return False
    return True
