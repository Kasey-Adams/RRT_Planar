import matplotlib.pyplot as plt
import matplotlib.patches as patch
import math
import random
import numpy as np
from scipy.integrate import solve_ivp
import time


class Environment:
    def __init__(self):

        # obstacle array of [x,y,r] arrays defining obstacles in teh simulation
        self.obstacle = obstacles
        # goal region
        self.xg = goal[0]
        self.yg = goal[2]
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
    # def through_obstacle(self, ax, bx, ay, by):
    #     collision = False
    #     lab = np.sqrt((bx - ax) ** 2 + (by - ay) ** 2)  # length of ray a-b
    #     # print(lab)
    #     if lab != 0:
    #         dx = (bx - ax) / lab  # x direction vector componenet
    #         dy = (by - ay) / lab  # y direction vector componenet
    #         for i in range(0, len(self.obstacle)):
    #             cx = self.obstacle[i][0]  # x coordinate of obstacle center
    #             cy = self.obstacle[i][1]  # y coordinate of obstacle center
    #             cr = self.obstacle[i][2]  # radius of obstacle
    #             t = dx * (cx - ax) + dy * (cy - ay)  # distance between point a and closest point to obstacle on ray
    #             ex = t * dx + ax  # x coords for closest point to circle
    #             ey = t * dy + ay  # y coords for closest point to circle
    #             lec = np.sqrt((ex - cx) ** 2 + (ey - cy) ** 2)  # distance between e and obstacle center
    #             if lec <= (cr + radius):
    #                 collision = True
    #     # print(collision)
    #     return collision

    # checks if point is in goal
    def in_goal(self, x, y):
        dpg = np.sqrt((x - self.xg) ** 2 + (y - self.yg) ** 2)  # distance from point to goal center
        if dpg < self.eg:  # if distance is less than goal radius end
            return True
        return False

    def in_bounds(self, x, y):
        if x < self.xmin or x > self.xmax or y < self.ymin or y > self.ymax:
            return False
        return True


class RRT:
    def __init__(self):
        # starting node
        nstart = initial_position
        self.state = [[], [], [], [], [], []]
        self.parent = []
        self.state[0].append(nstart[0])
        self.state[1].append(nstart[1])
        self.state[2].append(nstart[2])
        self.state[3].append(nstart[3])
        self.state[4].append(nstart[4])
        self.state[5].append(nstart[5])

        self.time = []
        # first node is the only node whose parent is itself
        self.parent.append(0)
        self.time.append(0)

        se;f.dmax = dmax

        self.goalstate = None
        self.path = []

    def distance_between(self, n1, n2):
        d = np.sqrt((self.state[0][n1] - self.state[0][n2]) ** 2 + (self.state[2][n1] - self.state[2][n2]) ** 2)
        return d

    # expand a random node and test if its valid, connect to nearest node if it is
    def expand(self):
        x = np.zeros(6)
        in_obs = True
        while in_obs is True:
            x[0] = np.random.randn() * self.dmax + E.xg
            x[1] = np.random.randn() * vel_var
            x[2] = np.random.randn() * self.dmax + E.yg
            x[3] = np.random.randn() * vel_var
            x[4] = np.random.rand() * 2 * np.pi
            x[5] = np.random.randn() * vel_var
            if E.in_obstacle(x[0], x[2]) is False and E.in_bounds(x[0], x[2]) is True:
                in_obs = False
            # print(in_obs)
        dt = np.random.rand()
        n = self.number_of_nodes()
        self.add_node(n, x)
        # print(self.state)
        n_nearest = self.near(n)
        n_nearest = int(n_nearest)
        x_nearest = []
        for i in range(0, 6):
            x_nearest.append(self.state[i][n_nearest])
        # print(x_nearest)
        # print(x)
        nearest_parent = self.parent[n_nearest]
        # print(nearest_parent)
        t_nearest = self.time[nearest_parent]
        (x_new, u, col) = self.steer(x_nearest, x, t_nearest, t_nearest + dt)
        # print(x_new)
        self.remove_node(n)
        if col is True:
            return
        else:
            self.add_node(n, x_new)
            self.add_edge(n_nearest, n)
            self.time.insert(n, t_nearest + dt)
            x_check = np.ma.array(x_new, mask=False)
            x_check.mask[4] = True
            self.dmax = np.linalg.norm(x_check.compressed() - goal)

        # ncheckb = self.number_of_nodes()  # check nodes before
        # self.step(nearest, n)
        # nchecka = self.number_of_nodes()  # check if nodes were removed
        # # print(ncheckb, nchecka)
        # if nchecka != ncheckb:
        #     return

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
    # def step(self, near, new):
    #     d = self.distance_between(near, new)
    #     # print(d)
    #     # print(d)
    #     if d > dmax:  # if the step size is too great, lower step size
    #         theta = math.atan2(self.y[new] - self.y[near], self.x[new] - self.x[near])
    #         # print(theta)
    #         for i in range(0, 5):
    #             (xn, yn) = (
    #                 self.x[new] * math.cos(theta) * dmax * (5 - i) / 5,
    #                 self.y[new] * math.sin(theta) * dmax * (5 - i) / 5)
    #             if E.in_obstacle(xn, yn) is not True and E.through_obstacle(self.x[near], xn, self.y[near],
    #                                                                         yn) is not True:
    #                 self.remove_node(new)
    #                 # print(new,xn,yn)
    #                 self.add_node(new, xn, yn)
    #                 # print(self.distance_between(near,new))
    #                 return
    #         self.remove_node(new)
    #         return
    #         # if no step can be found in the step direction, place node on top of nearest node to ensure

    def add_node(self, n, x):
        for i in range(0, 6):
            self.state[i].insert(n, x[i])

    def remove_node(self, n):
        for i in range(0, 6):
            self.state[i].pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def clear(self):
        nstart = initial_position
        self.state = [[], [], [], [], [], []]
        self.parent = []
        self.state[0].append(nstart[0])
        self.state[1].append(nstart[1])
        self.state[2].append(nstart[2])
        self.state[3].append(nstart[3])
        self.state[4].append(nstart[4])
        self.state[5].append(nstart[5])

        # first node is the only node whose parent is itself
        self.parent.append(0)
        self.goalstate = None
        self.path = []

    def number_of_nodes(self):
        return len(self.state[0])

    # draw tree
    def showtree(self, k):
        # print(len(self.state[0]))
        # print(len(self.parent))
        for i in range(0, self.number_of_nodes()):
            par = self.parent[i]
            plt.plot([self.state[0][i], self.state[0][par]], [self.state[2][i], self.state[2][par]], k, lw=0.5)

    # draw path
    def showpath(self, k):
        current = self.number_of_nodes() - 1
        parent = self.parent[current]
        while current != 0:
            plt.plot([self.state[0][current], self.state[0][parent]], [self.state[2][current], self.state[2][parent]],
                     k, lw=2)
            current = parent
            parent = self.parent[current]

    def steer(self, x0, x1, t0, tf):
        n_samples = 50
        u_candidates = []
        x_candidates = []
        col_list = []
        for i in range(0, n_samples):
            u_candidates.append([0, 0])
            x_candidates.append([0, 0, 0, 0, 0, 0])
            col_list.append(True)
        # print(x_candidates)
        x_free = []
        u_free = []
        for i in range(0, n_samples):
            u_candidates[i] = self.sample_u()
            x_candidates[i], col_list[i] = self.propegate_dynamics(x0, u_candidates[i], t0, tf)
        for i in range(0, len(col_list)):
            if col_list[i] is False:
                x_free.append(x_candidates[i])
                u_free.append(u_candidates[i])
        if x_free == []:
            return None, None, True
        else:
            nearest = 0
            dist = np.sqrt((x_free[0][0] - x1[0]) ** 2 + (x_free[0][2] - x1[2]) ** 2)
            for i in range(1, len(x_free)):
                dist1 = np.sqrt((x_free[i][0] - x1[0]) ** 2 + (x_free[i][2] - x1[2]) ** 2)
                if dist1 < dist:
                    dist = dist1
                    nearest = i
            x_new = x_free[nearest]
            u_new = u_free[nearest]
            return x_new, u_new, False

    def sample_u(self):
        u = np.zeros(2)
        u[0] = np.random.rand()
        u[1] = np.random.randn()
        return u

    def propegate_dynamics(self, x0, u, t0, tf):
        def get_xdot(t, x):
            xdot = np.array([x[1],
                             u[0] * np.cos(x[4]),
                             x[3],
                             u[0] * np.sin(x[4]),
                             x[5],
                             u[1]])
            return xdot

        tsteps = []
        for i in range(0, steps):
            tsteps.append((tf - t0) * i / steps + t0)
        # print(x0)
        sol = solve_ivp(get_xdot, [t0, tf], x0, t_eval=tsteps)
        xout = sol.y
        xnew = []
        for i in range(0, 6):
            xnew.append(xout[i][-1])
        collision = False
        # print(xout)

        for i in range(0, steps):
            if E.in_obstacle(xout[0][i], xout[2][i]) is True or E.in_bounds(xout[0][i], xout[2][i]) is False:
                collision = True
                # print(i)
                break
            # if E.in_goal(xout[0][i], xout[2][i]):
            #     for j in range(0, 6):
            #         xnew[j] = xout[j][i]
            #         break

        return xnew, collision


# Global Variables
radius = .5  # radius of bot

# node limit
nmax = 250000

# integration steps
steps = 6

# goal region
initial_position = np.zeros(6)
goal = np.array([10, 0, 0, 0, 0, 0])
eg = .25

# simulation boundaries
xmin = -5
xmax = 15
ymin = -10
ymax = 10

# extend step size
dmax = 10

# velocity variance
vel_var = 2

# obstacles
obstacles = [[5, 0, 1]]

# create an RRT tree with a start node
G = RRT()

# environment instance
E = Environment()


def draw(goalstate):
    # draw bounds
    fig, ax = plt.add_subplot()
    plt.plot([xmin, xmin, xmax, xmax, xmin], [ymin, ymax, ymax, ymin, ymin], color='k', lw=.5)
    # goal region
    goal = plt.Circle((E.xg, E.yg), radius=eg, color='g')
    ax.add_artist(goal)
    # draw tree
    G.showtree('0.45')
    # draw path
    print(goalstate)
    if goalstate < nmax + 1:
        G.showpath('r-')

    # draw obstacles
    for obstacle in obstacles:
        obs = plt.Circle((obstacle[0], obstacle[1]), radius=obstacle[2], color='k', fill=None)
        ax.add_artist(obs)
    plt.show()


def main():
    goalstate = nmax + 1
    for i in range(0, nmax):
        G.expand()
        if i % 1000 == 0:
            print(i)
        if E.in_goal(G.state[0][-1], G.state[2][-1]):
            goalstate = i
            break
    plt.text(45, 103, 'Loops: %d' % (goalstate + 1))
    # print(G.x)
    # print(G.y)
    draw(goalstate)


# run main when RRT is called
if __name__ == '__main__':
    main()
