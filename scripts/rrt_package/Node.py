import math


class Node:
    def __init__(self, pos=(0, 0), parent=None):

        if len(pos) == 2:
            pos = tuple(pos)
        else:
            raise ValueError(" (pos) should be a size 2 tuple")

        self.pos = pos
        self.parent = parent

    def getParent(self):
        return self.parent

    def getPos(self, coord=None):
        if not coord:
            return self.pos
        elif coord == "x":
            return self.pos[0]
        elif coord == "y":
            return self.pos[1]
        else:
            raise ValueError(f"{coord} should be 'x' or 'y'")

    def getX(self):
        return self.pos[0]

    def getY(self):
        return self.pos[1]

    def distance_from(self, pos):
        return math.dist(self.pos, pos)
