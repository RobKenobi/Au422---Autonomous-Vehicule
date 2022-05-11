import sys
from time import time

import matplotlib.pyplot as plt
import numpy as np

from rrt_package.Node import Node
from rrt_package.utils import gen_point, nearest, checkpoints, plot_path_coord

DEFAULT_MAP = grid1 = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
                                 1, 1, 1, 1, 1, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0,
                                 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                 1, 1, 1, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 1, 1, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 1, 1, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
                                 1, 1, 1, 1, 1, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])


class Path:
    def __init__(self, initial_node=Node(), destination_node=Node(), delta_q=1, environnement=DEFAULT_MAP, max_iter=100, debug=False):
        self.init_node = initial_node
        self.dest_node = destination_node
        self.dq = delta_q
        self.env = environnement
        self.optimized = False
        self.max_iter = max_iter

        self._path = []
        self._optimized_path = []

        print("Path has been created, do 'Path'.generate() to generate and 'Path'.display() to see the calculated path")

    def _new_node(self, list_node):
        obstacle = 1
        width, height = self.env.shape
        new_point = gen_point(width, height)
        nearest_q = nearest(list_node, new_point)
        x_parent = nearest_q.getX()
        y_parent = nearest_q.getY()

        # if the distance between the nearest point and the goal is inferior to delta_q
        if nearest_q.distance_from(self.dest_node.getPos()) <= self.dq:
            x, y = self.dest_node.getPos()
        else:
            # Compute the angle between the nearest q and the random position generated
            theta = np.arctan2(new_point[1] - y_parent, new_point[0] - x_parent)

            # Compute coordinates of the new point

            # x must be an int between 0 and width-1
            x = max(0, min(round(x_parent + (self.dq * np.cos(theta))), width - 1))
            # y must be an int between 0 and height-1
            y = max(0, min(round(y_parent + (self.dq * np.sin(theta))), height - 1))

            # Checking that the new point is not on an obstacle
            if self.env[x, y] == obstacle:
                return None

        # Checking that there are no obstacle between nearest_q and the new point
        #if not self.pathLineVerif((x, y), (x_parent, y_parent), 5):
        #    return None

        return Node((x, y), nearest_q)

    def pathLineVerif(self, pos1, pos2, rob):
        width, height = self.env.shape

        def saturator_x(x):
            return min(width - 1, max(0, x))

        def saturator_y(y):
            return min(height - 1, max(0, y))

        def square(x, y):
            return self.env[saturator_x(x - rob // 2):saturator_x(x + rob // 2)][
                   saturator_y(y - rob // 2):saturator_y(y + rob // 2)]

        def verif(x, y):
            table = list(np.array(square(x, y)).flatten())
            return table.count(1) == 0

        rx, ry = checkpoints(pos1[0], pos1[1], pos2[0], pos2[1])

        if rx is None:
            print("WTF!")
            return False

        n = len(rx)
        print(n)
        for i in range(n):
            if not verif(rx[i], ry[i]):
                return False
        return False

    def generate(self, optimized=False):
        node = self.init_node
        list_nodes = [node]
        while node.getPos() != self.dest_node.getPos() and self.max_iter > 0:
            self.max_iter -= 1
            new_node = self._new_node(list_nodes)
            # print(new_node)
            if new_node is None:
                continue
            node = new_node
            list_nodes.append(node)

            # Affichage
            self.env[node.getX()][node.getY()] = 2
            pos_node = node.getPos()
            pos_parent = node.getParent().getPos()
            x = [pos_node[1], pos_parent[1]]
            y = [pos_node[0], pos_parent[0]]
            plt.plot(x, y, 'c--')

        if not self.max_iter:
            print("MAX")
            exit(1)

        path = list()
        while node.getParent():
            path.append(node)
            self.env[node.getX(), node.getY()] = 3
            node = node.getParent()
        self.env[self.init_node.getPos("x"), self.init_node.getPos("y")] = 4
        self.env[self.dest_node.getPos("x"), self.dest_node.getPos("y")] = 4
        path.append(self.init_node)
        path.reverse()
        self._path = path

        if optimized:
            self.optimized = optimized
            obstacle = 1
            optimized_path = [path[0]]

            path_copy = path.copy()
            prev_node = path_copy[0]
            path_copy.pop(0)

            for pos_node in path_copy:
                x1, y1 = pos_node.getPos()
                x2, y2 = optimized_path[-1].getPos()
                x_2_check, y_2_check = checkpoints(x1, y1, x2, y2)
                if x_2_check is None:
                    continue
                for i in range(len(x_2_check)):
                    if self.env[x_2_check[i], y_2_check[i]] == obstacle:
                        optimized_path.append(prev_node)
                        break
                prev_node = pos_node
            optimized_path.append(path_copy[-1])
            self._optimized_path = optimized_path

    def display(self):
        t0 = time()
        x, y = plot_path_coord(self._path)
        if self.optimized:
            x_opti, y_opti = plot_path_coord(self._optimized_path)

        # print(f'Original path : {path}')
        # print(f'Optimized path :{optimized_path}')

        # Affichage
        plt.imshow(self.env)
        plt.plot(x, y, 'y', label="Original Path")

        if self.optimized:
            plt.plot(x_opti, y_opti, 'r', label="Optimized Path")
        plt.legend(loc='best')
        plt.title(f"Path with $\Delta_q = ${self.dq}")
        plt.show()

        print(f"Displaying time : {round(time() - t0, 2)} s", file=sys.stderr)


if __name__ == "__main__":
    print("Starting RRT")
    P = Path(Node((0, 0)), Node((10, 10)), delta_q=4, environnement=DEFAULT_MAP, max_iter=1000)
    P.generate(optimized=True)
    P.display()
