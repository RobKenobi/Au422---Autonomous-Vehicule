from Node import Node
from Map import DEFAULT_MAP
import numpy as np
import matplotlib.pyplot as plt


def gen_point(width, height):
    return np.random.randint(width), np.random.randint(height)


def saturation(value, maximum, minimum=0):
    return max(min(value, maximum), minimum)


class Path:
    i = 0

    def __init__(self, init_node=Node(), goal_node=Node(), map_env=DEFAULT_MAP, dq=2, robot_size=3, max_iter=1000):
        if isinstance(init_node, Node):
            self.init_node = init_node
        else:
            raise TypeError("init_node type must be Node")

        if isinstance(goal_node, Node):
            self.goal_node = goal_node
        else:
            raise TypeError("goal_node type must be Node")

        try:
            if len(map_env.shape) == 2:
                self.map = map_env
                self.plot_map = map_env
            else:
                raise ValueError("map_env must be a 2D-array")
        except:
            raise TypeError("map_env type must be numpy.ndarray")

        if isinstance(dq, int):
            self.dq = dq
        else:
            raise ValueError("qd type must be int")

        if isinstance(robot_size, int):
            self.robot_size = robot_size
        else:
            raise ValueError("robot_size type must be int")

        if isinstance(max_iter, int):
            self.max_iter = max_iter
        else:
            raise ValueError("max_iter type must be int")

        self.list_nodes = [self.init_node]
        self.obstacle = 1
        self.optimized = False
        self.original_path = list()
        self.optimized_path = list()

    def free_square(self, x, y):
        width, height = self.map.shape
        x_min = x - self.robot_size
        x_max = x + self.robot_size + 1

        y_min = y - self.robot_size
        y_max = y + self.robot_size + 1

        if x_min < 0 or y_min < 0 or x_max > width or y_max > height:
            return False

        square = self.map[x_min:x_max, y_min:y_max]

        if np.sum(square):
            return False
        return True

    def no_obstacle(self, pos1, pos2):
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
            return None

        for x, y in zip(to_check_x, to_check_y):
            if not self.free_square(x, y):
                return False
        return True

    def new_node(self, list_nodes, plot=False):
        width, height = self.map.shape

        # Generating new point
        new_point = gen_point(width, height)
        nearest_point = min(list_nodes, key=lambda node: node.distance_from(new_point))

        # Saving coordinates of parent node
        x_parent = nearest_point.getPos('x')
        y_parent = nearest_point.getPos('y')

        # Checking the goal if the goal is in a range inferior to dq
        if nearest_point.distance_from(self.goal_node.getPos()) < self.dq:
            x, y = self.goal_node.getPos()

        else:
            # Computing the angle between new_point and nearest_point
            x_new, y_new = new_point
            theta = np.arctan2(y_new - y_parent, x_new - x_parent)

            x = saturation(int(x_parent + (self.dq * np.cos(theta))), width - 1)
            y = saturation(int(y_parent + (self.dq * np.sin(theta))), height - 1)

            # Checking that the new point is not near an obstacle
            if not self.free_square(x, y):
                return None

            if not self.no_obstacle((x, y), (x_parent, y_parent)):
                return None

        if plot:
            plt.plot([y, y_parent], [x, x_parent], 'c--')
            self.plot_map[x, y] = 2

        return Node((x, y), nearest_point)

    def optimize(self):
        # TODO implémenter l'optimisation
        pass

    def generate(self, optimize=False, plot=False):
        node = self.init_node
        while node.getPos() != self.goal_node.getPos() and self.max_iter > 0:
            self.max_iter -= 1
            new_node = self.new_node(self.list_nodes, plot=plot)
            if new_node is not None:
                node = new_node
                self.list_nodes.append(node)


        if self.max_iter:
            while node.getParent() is not None:
                self.original_path.append(node.getPos())
                node = node.getParent()
            self.original_path.reverse()

            if plot:
                x = [self.init_node.getPos('x')]
                y = [self.init_node.getPos('y')]
                self.plot_map[x,y] = 3
                for x_i, y_i in self.original_path:
                    x.append(x_i)
                    y.append(y_i)
                    self.plot_map[x_i, y_i] = 3
                plt.plot(y, x, 'y-')
                plt.imshow(self.plot_map)
                plt.show()
            if optimize:
                self.optimize()
        else:
            print("MAX")

    def getPath(self, optimize=False):
        if optimize:
            if self.optimized:
                return self.optimized_path
            else:
                self.optimize()
                return self.optimized_path
        return self.original_path


map = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0],
                [0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

if __name__ == "__main__":
    init = Node((1, 1))
    goal = Node((17, 17))
    P = Path(init, goal, map_env=DEFAULT_MAP, dq=4, robot_size=3, max_iter=100000)
    P.generate(optimize=False, plot=True)
    print(P.getPath(optimize=False))
    print(P.list_nodes)
