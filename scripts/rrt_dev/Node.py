import math


class Node:
    def __init__(self, pos=(0, 0), parent=None):
        if isinstance(pos, tuple):
            if len(pos) == 2:
                for x in pos:
                    if not isinstance(x, int):
                        raise ValueError("pos must be a tuple of int")
                self.pos = pos
            else:
                raise ValueError("pos should be a tuple of size 2")
        else:
            raise TypeError("pos type should be tuple")

        if isinstance(parent, Node):
            self.parent = parent
        elif parent is None:
            self.parent = None
        else:
            raise TypeError("parent type should be Node")

    def getParent(self):
        return self.parent

    def getPos(self, coord=None):
        if coord is None:
            return self.pos
        if coord == "x":
            return self.pos[0]
        if coord == "y":
            return self.pos[1]
        raise ValueError(f"{coord} should be 'x' or 'y'")

    def distance_from(self, pos):
        if isinstance(pos, tuple):
            if len(pos) == 2:
                return math.dist(self.pos, pos)
            raise ValueError("pos should be a tuple of size 2")
        raise TypeError("pos type should be tuple")


if __name__ == "__main__":
    a = Node((10, 10))
    b = Node((5, 5), a)

    print(f"a.getPos('x') : {a.getPos('x')}")
    print(f"a.getPos('y') : {a.getPos('y')}")

    print(f"a.getParent() : {a.getParent()}")
    print(f"b.getParent() == a : {b.getParent() == a}")

    print(f"a.distance_from(b.getPos()) : {a.distance_from(b.getPos())}")