import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import math
import random
import rowan
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
        self.zg = goal[4]
        self.eg = eg


        # simulation boundaries
        self.xmin = xmin
        self.ymin = ymin
        self.zmin = zmin
        self.xmax = xmax
        self.ymax = ymax
        self.zmax = zmax

    # check if point is in obstacle
    def in_obstacle(self, x, y, z):
        obs = False
        for i in range(0, len(self.obstacle)):
            cx = self.obstacle[i][0]  # x coordinate of obstacle center
            cy = self.obstacle[i][1]  # y coordinate of obstacle center
            cz = self.obstacle[i][2]  # z coordinate of obstacle center
            cr = self.obstacle[i][3]  # radius of obstacle
            acd = np.sqrt((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2)
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
    def in_goal(self, x, y, z):
        dpg = np.sqrt(
            (x - self.xg) ** 2 + (y - self.yg) ** 2 + (z - self.zg) ** 2)  # distance from point to goal center
        if dpg < self.eg:  # if distance is less than goal radius end
            return True
        return False

    def in_bounds(self, x, y, z):
        if x < self.xmin + radius or x > self.xmax - radius or y < self.ymin + radius or y > self.ymax - radius or z < self.zmin + radius or z > self.zmax - radius:
            return False
        return True


class RRT:
    def __init__(self):
        # starting node
        nstart = initial_position
        self.state = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.parent = []
        for i in range(0, len(self.state)):
            self.state[i].append(nstart[i])

        self.time = []
        # first node is the only node whose parent is itself
        self.parent.append(0)
        self.time.append(0)

        self.dmax = dmax

        self.goalstate = None
        self.path = []

    def distance_between(self, n1, n2):
        d = np.sqrt((self.state[0][n1] - self.state[0][n2]) ** 2 + (self.state[2][n1] - self.state[2][n2]) ** 2 + (
                self.state[4][n1] - self.state[4][n2]) ** 2)
        return d

    # expand a random node and test if its valid, connect to nearest node if it is
    def expand(self):
        x = np.zeros(12)
        in_obs = True
        while in_obs is True:
            x[0] = np.random.randn() * self.dmax + E.xg  # x
            x[1] = np.random.randn() * vel_var  # dxdt
            x[2] = np.random.randn() * self.dmax + E.yg  # y
            x[3] = np.random.randn() * vel_var  # dydt
            x[4] = np.random.randn() * self.dmax + E.zg  # z
            x[5] = np.random.randn() * vel_var  # dzdt
            x[6] = np.random.rand() * 2 * np.pi  # yaw
            x[7] = np.random.randn() * vel_var  # dyawdt
            x[8] = np.random.rand() * 2 * np.pi  # pitch
            x[9] = np.random.randn() * vel_var  # dpitchdt
            x[10] = np.random.rand() * 2 * np.pi  # roll
            x[11] = np.random.randn() * vel_var  # drolldt
            if E.in_obstacle(x[0], x[2], x[4]) is False and E.in_bounds(x[0], x[2], x[4]) is True:
                in_obs = False
            # print(in_obs)
        dt = np.random.rand()
        n = self.number_of_nodes()
        self.add_node(n, x)
        # print(self.state)
        n_nearest = self.near(n)
        n_nearest = int(n_nearest)
        x_nearest = []
        for i in range(0, 12):
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
            x_check.mask[6] = True
            x_check.mask[8] = True
            x_check.mask[10] = True
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
        for i in range(0, 12):
            self.state[i].insert(n, x[i])

    def remove_node(self, n):
        for i in range(0, 12):
            self.state[i].pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def clear(self):
        nstart = initial_position
        self.state = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.parent = []
        for i in range(0, len(self.state)):
            self.state[i].append(nstart[i])
        # first node is the only node whose parent is itself
        self.parent.append(0)
        self.goalstate = None
        self.path = []

    def number_of_nodes(self):
        return len(self.state[0])

    # draw tree
    # def showtree(self, k):
    #     # print(len(self.state[0]))
    #     # print(len(self.parent))
    #     for i in range(0, self.number_of_nodes()):
    #         par = self.parent[i]
    #         ax.plot3D([self.state[0][i], self.state[0][par]], [self.state[2][i], self.state[2][par]],
    #                    [self.state[4][i], self.state[4][par]], k, lw=0.5)

    # draw path
    # def showpath(self, k):
    #     current = self.number_of_nodes() - 1
    #     parent = self.parent[current]
    #     while current != 0:
    #         ax.plot3D([self.state[0][current], self.state[0][parent]], [self.state[2][current], self.state[2][parent]],
    #                    [self.state[4][current], self.state[4][parent]],
    #                    k, lw=2)
    #         current = parent
    #         parent = self.parent[current]

    def steer(self, x0, x1, t0, tf):
        n_samples = 50
        u_candidates = []
        x_candidates = []
        col_list = []
        for i in range(0, n_samples):
            u_candidates.append([0, 0, 0, 0, 0, 0])
            x_candidates.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
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
            dist = np.sqrt((x_free[0][0] - x1[0]) ** 2 + (x_free[0][2] - x1[2]) ** 2 + (x_free[0][4] - x1[4]) ** 2)
            for i in range(1, len(x_free)):
                dist1 = np.sqrt((x_free[i][0] - x1[0]) ** 2 + (x_free[i][2] - x1[2]) ** 2 + (x_free[i][4] - x1[4]) ** 2)
                if dist1 < dist:
                    dist = dist1
                    nearest = i
            x_new = x_free[nearest]
            u_new = u_free[nearest]
            return x_new, u_new, False

    def sample_u(self):
        u = np.zeros(6)
        u[0] = np.random.rand()
        u[1] = np.random.rand()
        u[2] = np.random.rand()
        u[3] = np.random.randn()
        u[4] = np.random.randn()
        u[5] = np.random.randn()
        return u

    def propegate_dynamics(self, x0, u, t0, tf):
        def get_xdot(t, x):
            xdot = np.array([x[1],
                             u[0] * np.cos(x[6]),
                             x[3],
                             u[1] * np.sin(x[6]),
                             x[5],
                             u[2] * np.sin(x[8]),
                             x[7],
                             u[3],
                             x[9],
                             u[4],
                             x[11],
                             u[5]])
            return xdot

        tsteps = []
        for i in range(0, steps):
            tsteps.append((tf - t0) * i / steps + t0)
        # print(x0)
        sol = solve_ivp(get_xdot, [t0, tf], x0, t_eval=tsteps)
        xout = sol.y
        xnew = []
        for i in range(0, 12):
            xnew.append(xout[i][-1])
        collision = False
        # print(xout)

        for i in range(0, steps):
            if E.in_obstacle(xout[0][i], xout[2][i], xout[4][i]) is True or E.in_bounds(xout[0][i], xout[2][i],
                                                                                        xout[4][i]) is False:
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
nmax = 100000

# integration steps
steps = 6

# goal region
initial_position = np.zeros(12)
goal = np.array([10, 0, 0, 0, 0, 0, 0, 0, 0])
eg = 0.25

# simulation boundaries
xmin = -5
xmax = 15
ymin = -10
zmin = -10
ymax = 10
zmax = 10
# extend step size
dmax = 10

# velocity variance
vel_var = 2

# obstacles
obstacles = [[5, 0, 0, 1]]

# create an RRT tree with a start node
G = RRT()

# environment instance
E = Environment()


def draw(goalstate):
    # draw bounds
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d([xmin, xmax])
    ax.set_xlabel('X')
    ax.set_ylim3d([ymin, ymax])
    ax.set_ylabel('Y')
    ax.set_zlim3d([zmin, zmax])
    ax.set_zlabel('Z')
    # plt.plot([xmin, xmin, xmax, xmax, xmin], [ymin, ymax, ymax, ymin, ymin], color='k', lw=.5)
    # goal region
    for num in range(100):
        r = eg
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = r * np.outer(np.cos(u), np.sin(v)) + E.xg
        y = r * np.outer(np.sin(u), np.sin(v)) + E.yg
        z = r * np.outer(np.ones(np.size(u)), np.cos(v)) + E.zg
        sphere = ax.plot_surface(x, y, z, color='g')
    # draw tree
    for i in range(0, G.number_of_nodes()):
        par = G.parent[i]
        ax.plot3D([G.state[0][i], G.state[0][par]], [G.state[2][i], G.state[2][par]],
                  [G.state[4][i], G.state[4][par]], '0.45', lw=0.5)
    # draw path
    print(goalstate)
    if goalstate < nmax + 1:
        current = G.number_of_nodes() - 1
        parent = G.parent[current]
        while current != 0:
            ax.plot3D([G.state[0][current], G.state[0][parent]], [G.state[2][current], G.state[2][parent]],
                       [G.state[4][current], G.state[4][parent]],
                       'r-', lw=2)
            current = parent
            parent = G.parent[current]

    # draw obstacles
    for obstacle in obstacles:
        for num in range(100):
            r = obstacle[3]
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            x = r * np.outer(np.cos(u), np.sin(v)) + obstacle[0]
            y = r * np.outer(np.sin(u), np.sin(v)) + obstacle[1]
            z = r * np.outer(np.ones(np.size(u)), np.cos(v)) + obstacle[2]
            sphere = ax.plot_surface(x, y, z, color='k')
    plt.show()


def main():
    goalstate = nmax + 1
    for i in range(0, nmax):
        G.expand()
        if i % 1000 == 0:
            print(i)
        if E.in_goal(G.state[0][-1], G.state[2][-1], G.state[4][-1]):
            goalstate = i
            break
    plt.text(45, 103, 'Loops: %d' % (goalstate + 1))
    # print(G.x)
    # print(G.y)
    draw(goalstate)


# run main when RRT is called
if __name__ == '__main__':
    main()
