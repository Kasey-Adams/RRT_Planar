import matplotlib.pyplot as plt
import matplotlib.patches as patch
import math
import random
import numpy as np


class Environment:
    def __init__(self):

        # obstacle array of [x,y,r] arrays defining obstacles in teh simulation
        self.obstacle = obstacles
        # goal region
        self.xg = xg
        self.yg = yg
        self.eg = eg

        # simulation boundaries
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax

    # check if point is in obstacle
    def in_obstacle(self, x, y):
        obs = False
        for i in range(0, len(self.obstacle)):
            cx = self.obstacle[i][0]  # x coordinate of obstacle center
            cy = self.obstacle[i][1]  # y coordinate of obstacle center
            cr = self.obstacle[i][2]  # radius of obstacle
            acd = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
            if acd <= cr + radius:
                obs = True
        return obs

    # check if the path between two points goes through an obstacle
    def through_obstacle(self, ax, bx, ay, by):
        collision = False
        lab = np.sqrt((bx - ax) ** 2 + (by - ay) ** 2)  # length of ray a-b
        # print(lab)
        if lab != 0:
            dx = (bx - ax) / lab  # x direction vector componenet
            dy = (by - ay) / lab  # y direction vector componenet
            for i in range(0, len(self.obstacle)):
                cx = self.obstacle[i][0]  # x coordinate of obstacle center
                cy = self.obstacle[i][1]  # y coordinate of obstacle center
                cr = self.obstacle[i][2]  # radius of obstacle
                t = dx * (cx - ax) + dy * (cy - ay)  # distance between point a and closest point to obstacle on ray
                ex = t * dx + ax  # x coords for closest point to circle
                ey = t * dy + ay  # y coords for closest point to circle
                lec = np.sqrt((ex - cx) ** 2 + (ey - cy) ** 2)  # distance between e and obstacle center
                if lec <= (cr + radius):
                    collision = True
        # print(collision)
        return collision

    # checks if point is in goal
    def in_goal(self, x, y):
        dpg = np.sqrt((x - self.xg) ** 2 + (y - self.yg) ** 2)  # distance from point to goal center
        if dpg < self.eg:  # if distance is less than goal radius end
            return True
        return False

    # def in_bounds(self, x, y):
    #     if x < self.xmin or x > self.xmax or y < self.ymin or y > self.ymax:
    #         return False
    #     return True


class RRT:
    def __init__(self):
        # starting node
        (xs, ys) = nstart
        self.x = []
        self.y = []
        self.parent = []
        self.x.append(xs)
        self.y.append(ys)
        # first node is the only node whose parent is itself
        self.parent.append(0)
        self.goalstate = None
        self.path = []

    def distance_between(self, n1, n2):
        d = np.sqrt((self.x[n1] - self.x[n2]) ** 2 + (self.y[n1] - self.y[n2]) ** 2)
        return d

    # expand a random node and test if its valid, connect to nearest node if it is
    def expand(self):
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        n = self.number_of_nodes()
        self.add_node(n, x, y)
        nearest = self.near(n)
        ncheckb = self.number_of_nodes()  # check nodes before
        self.step(nearest, n)
        nchecka = self.number_of_nodes()  # check if nodes were removed
        # print(ncheckb, nchecka)
        if nchecka != ncheckb:
            return
        if E.in_obstacle(self.x[n], self.y[n]) is not True:
            self.add_edge(nearest, n)
        else:
            self.remove_node(n)

    # find the nearest node
    def near(self, n):
        dmin = self.distance_between(0, n)
        # print(dmin)
        nearest = 0
        for i in range(1, n):
            # print(self.distance_between(i, n))
            #  print(dmin)
            if self.distance_between(i, n) < dmin:
                dmin = self.distance_between(i, n)
                nearest = i
        # print(nearest)
        return nearest

    # if step size is greater than maximum step size, scale down
    def step(self, near, new):
        d = self.distance_between(near, new)
        # print(d)
        # print(d)
        if d > dmax:  # if the step size is too great, lower step size
            theta = math.atan2(self.y[new] - self.y[near], self.x[new] - self.x[near])
            # print(theta)
            for i in range(0, 5):
                (xn, yn) = (
                    self.x[new] * math.cos(theta) * dmax * (5 - i) / 5,
                    self.y[new] * math.sin(theta) * dmax * (5 - i) / 5)
                if E.in_obstacle(xn, yn) is not True and E.through_obstacle(self.x[near], xn, self.y[near],
                                                                            yn) is not True:
                    self.remove_node(new)
                    # print(new,xn,yn)
                    self.add_node(new, xn, yn)
                    # print(self.distance_between(near,new))
                    return
            self.remove_node(new)
            return
            # if no step can be found in the step direction, place node on top of nearest node to ensure

    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.insert(n, y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def clear(self):
        (x, y) = nstart
        self.x = []
        self.y = []
        self.parent = []
        self.x.append(x)
        self.y.append(y)
        # first node is the only node whose parent is itself
        self.parent.append(0)
        self.goalstate = None
        self.path = []

    def number_of_nodes(self):
        return len(self.x)

    def goal_path(self):
        for i in range(0, G.number_of_nodes()):
            if E.in_goal(self.x[i], self.y[i]):
                self.goalstate = i
                break
        if self.goalstate is not None:
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while newpos != 0:
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
            self.path.reverse()

    # draw tree
    def showtree(self, k):
        print(len(self.x))
        print(len(self.parent))
        for i in range(0, self.number_of_nodes()):
            par = self.parent[i]
            plt.plot([self.x[i], self.x[par]], [self.y[i], self.y[par]], k, lw=0.05)

    # draw path
    def showpath(self, k):
        for i in range(len(self.path) - 1):
            n1 = self.path[i]
            n2 = self.path[i + 1]
            plt.plot([self.x[n1], self.x[n2]], [self.y[n1], self.y[n2]], k, lw=1, markersize=3)


# Global Variables
radius = .5

# node limit
nmax = 50000

# goal region
xg = 10
yg = 0
eg = .25

# simulation boundaries
xmin = -5
xmax = 15
ymin = -10
ymax = 10

# extend step size
dmax = .5

# start the root of the tree
nstart = (0, 0)

# obstacles
obstacles = [[5, 0, 1]]

# create an RRT tree with a start node
G = RRT()

# environment instance

E = Environment()


def draw():
    # draw bounds
    fig, ax = plt.subplots()
    plt.plot([xmin, xmin, xmax, xmax, xmin], [ymin, ymax, ymax, ymin, ymin], color='k', lw=.5)
    # goal region
    goal = plt.Circle((xg, yg), radius=eg, color='g')
    ax.add_artist(goal)
    # draw tree
    G.showtree('0.45')
    # draw path
    G.showpath('r-')

    # draw obstacles
    for obstacle in obstacles:
        obs = plt.Circle((obstacle[0], obstacle[1]), radius=obstacle[2], color='k', fill=None)
        ax.add_artist(obs)
    plt.show()


def main():
    end = nmax
    for i in range(0, nmax):
        G.expand()
        if E.in_goal(G.x[-1], G.y[-1]):
            end = i
            break
    plt.text(45, 103, 'Loops: %d' % (end + 1))
    G.goal_path()
    # print(G.x)
    # print(G.y)
    draw()


# run main when RRT is called
if __name__ == '__main__':
    main()
