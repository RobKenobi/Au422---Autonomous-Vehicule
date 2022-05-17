from rrt_dev.Node import Node
import rrt_dev.Map
# from Node import Node
# import Map
import numpy as np
import sys


def gen_point(width, height):
    return np.random.randint(width), np.random.randint(height)


def saturation(value, maximum, minimum=0):
    return max(min(value, maximum), minimum)


class SmoothTurn:
    def __init__(self, points, alpha=0.9, nb_points=10):
        if not isinstance(points, list):
            raise TypeError("points must be a list of tuples")
        if len(points) != 3:
            raise ValueError("points must contain only three points")

        if not isinstance(nb_points, int):
            raise TypeError("nb_points must be an int")

        self.points = points
        self.x = [pos[0] for pos in self.points]
        self.y = [pos[1] for pos in self.points]
        self.nb_points = nb_points

        x1 = (1 - alpha) * self.x[0] + alpha * self.x[1]
        x2 = (1 - alpha) * self.x[2] + alpha * self.x[1]

        y1 = (1 - alpha) * self.y[0] + alpha * self.y[1]
        y2 = (1 - alpha) * self.y[2] + alpha * self.y[1]

        self.x = [x1, self.x[1], x2]
        self.y = [y1, self.y[1], y2]

        self.path = list()

    def generate(self):
        t = np.linspace(0, 1, self.nb_points)
        # Quadratic Bezier curve
        x_f = (1 - t) ** 2 * self.x[0] + 2 * t * (1 - t) * self.x[1] + t ** 2 * self.x[2]
        y_f = (1 - t) ** 2 * self.y[0] + 2 * t * (1 - t) * self.y[1] + t ** 2 * self.y[2]
        self.path = [(x, y) for x, y in zip(x_f, y_f)]
        return self.path


class Path:
    def __init__(self, init_node=Node(), goal_node=Node(), map_env=Map.big_map, dq=2, robot_size=3,
                 max_iter=1000):
        if not isinstance(init_node, Node):
            raise TypeError("init_node type must be Node")

        if not isinstance(goal_node, Node):
            raise TypeError("goal_node type must be Node")

        try:
            if len(map_env.shape) != 2:
                raise ValueError("map_env must be a 2D-array")
        except:
            raise TypeError("map_env type must be numpy.ndarray")

        if goal_node.getPos('x') < 0 or goal_node.getPos('x') > map_env.shape[1] or goal_node.getPos(
                'y') < 0 or goal_node.getPos('y') > map_env.shape[0]:
            raise ValueError(goal_node.getPos(), " is not on the map")

        if not isinstance(dq, int):
            raise ValueError("qd type must be int")

        if not isinstance(robot_size, int):
            raise ValueError("robot_size type must be int")

        if not isinstance(max_iter, int):
            raise ValueError("max_iter type must be int")

        self.init_node = init_node
        self.goal_node = goal_node

        self.dq = dq
        self.robot_size = robot_size

        self.max_iter = max_iter

        self.list_nodes = [self.init_node]
        self.optimized = False
        self.original_path = list()
        self.optimized_path = list()
        self.smooth_path = list()

        self.obstacle = 1
        self.map = map_env
        self.plot_map = self.map.copy()
        self.generate_free_space()

    def generate_free_space(self):
        x, y = np.where(self.map == self.obstacle)
        l, c = self.map.shape
        for i, j in zip(x, y):
            x_min = max(0, i - self.robot_size)
            x_max = min(l, i + self.robot_size + 1)
            y_min = max(0, j - self.robot_size)
            y_max = min(c, j + self.robot_size + 1)
            self.map[x_min:x_max, y_min:y_max] = self.obstacle

    def no_obstacle_on_line(self, pos1, pos2):
        x1, y1 = pos1
        x2, y2 = pos2

        if abs(x1 - x2) > abs(y1 - y2):
            to_check_x = np.arange(min(x1, x2) + 1, max(x1, x2))
            a = (y1 - y2) / (x1 - x2)
            b = y1 - a * x1
            to_check_y = [int(a * x_i + b) for x_i in to_check_x]

        elif abs(x1 - x2) < abs(y1 - y2):
            to_check_y = np.arange(min(y1, y2) + 1, max(y1, y2))
            a = (x1 - x2) / (y1 - y2)
            b = x1 - a * y1
            to_check_x = [int(a * y_i + b) for y_i in to_check_y]
        else:  # Meaning (x1,y1) == (x2,y2)
            return False

        for x, y in zip(to_check_x, to_check_y):
            if self.map[x, y] == self.obstacle:
                return False
        return True

    def new_node(self):
        width, height = self.map.shape

        # Generating new point
        new_point = gen_point(width, height)
        nearest_point = min(self.list_nodes, key=lambda node: node.distance_from(new_point))

        # Saving coordinates of parent node
        x_parent = nearest_point.getPos('x')
        y_parent = nearest_point.getPos('y')

        # Checking if the goal is in a range inferior to dq
        if nearest_point.distance_from(self.goal_node.getPos()) < self.dq:
            x, y = self.goal_node.getPos()

        else:
            # Computing the angle between new_point and nearest_point
            x_new, y_new = new_point
            theta = np.arctan2(y_new - y_parent, x_new - x_parent)

            x = saturation(int(x_parent + (self.dq * np.cos(theta))), width - 1)
            y = saturation(int(y_parent + (self.dq * np.sin(theta))), height - 1)

            # Checking that the new point is not near an obstacle
            if self.map[x, y] == self.obstacle:
                return None

            # Checking the path between the new node and its parent
            if not self.no_obstacle_on_line((x, y), (x_parent, y_parent)):
                return None

        return Node((x, y), nearest_point)

    def generate(self, optimize=False, smooth=False, alpha=0.75, nb_points=10):
        alpha = max(0.5, alpha)
        node = self.init_node
        while node.getPos() != self.goal_node.getPos() and self.max_iter:
            self.max_iter -= 1
            new_node = self.new_node()
            if new_node is not None:
                node = new_node
                self.list_nodes.append(node)

        if self.max_iter:
            while node.getParent() is not None:
                self.original_path.append(node.getPos())
                node = node.getParent()
            self.original_path.reverse()

            if optimize:
                self.optimize()

            if smooth:
                self.smoothPath(alpha, nb_points, optimized=optimize)
        else:
            print("Maximum iterations reached.\n"
                  "Possibles causes : \n"
                  " - The robot might be too big for the environment.\n"
                  " - The goal might be unreachable.\n", file=sys.stderr)

    def optimize(self):
        origin = self.init_node.getPos()
        path = self.original_path.copy()
        offset = 0
        while origin != self.goal_node.getPos():
            possibility = [path[offset]]
            for i in range(offset + 1, len(path)):
                if self.no_obstacle_on_line(origin, path[i]):
                    possibility.append(path[i])
                    offset = i + 1
            origin = possibility[-1]
            self.optimized_path.append(origin)
        self.optimized = True

    def smoothPath(self, alpha=0.75, nb_points=10, optimized=True):
        path_to_smooth = [self.original_path, self.optimized_path][optimized]
        prev = self.init_node.getPos()

        for i in range(len(path_to_smooth) - 1):
            valid = False
            while not valid:
                turn = SmoothTurn([prev, path_to_smooth[i], path_to_smooth[i + 1]], alpha, nb_points)
                p = turn.generate()
                for pos in p:
                    if self.map[round(pos[0]), round(pos[1])] == self.obstacle:
                        alpha = min(alpha + 0.1, 1)
                        break
                valid = True

            self.smooth_path.extend(p)
            prev = path_to_smooth[i]
        self.smooth_path.append(self.goal_node.getPos())

    def getPath(self, name='original', init=False):
        if name == "original" or name == "":
            path = self.original_path.copy()
        elif name == "optimized":
            path = self.optimized_path.copy()
        elif name == "smooth":
            path = self.smooth_path.copy()
        else:
            raise ValueError("name can only be {'original', 'optimized','smooth'}")

        if init:
            path.insert(0, self.init_node.getPos())
        return path


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    init = Node((1, 1))
    goal = Node((17, 17))
    map = rrt_dev.Map.DEFAULT_MAP1
    P = Path(init, goal, map_env=map, dq=6, robot_size=2, max_iter=10000)
    P.generate(optimize=True, smooth=True, alpha=0.5, nb_points=5)

    path = P.getPath("original", init=True)
    x = [pos[0] for pos in path]
    y = [pos[1] for pos in path]
    plt.plot(y, x, 'c', label="optimized")

    path = P.getPath("smooth", init=True)
    x = [pos[0] for pos in path]
    y = [pos[1] for pos in path]
    plt.plot(y, x, 'r', label="Smooth")

    plt.legend()

    plt.imshow(map)

    plt.show()
