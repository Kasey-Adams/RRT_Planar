import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import math
import random
import numpy as np
from scipy.integrate import solve_ivp


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

    # checks if point is in goal
    def in_goal(self, x, y, z):
        dpg = np.sqrt(
            (x - self.xg) ** 2 + (y - self.yg) ** 2 + (z - self.zg) ** 2)  # distance from point to goal center
        if dpg < self.eg:  # if distance is less than goal radius end
            return True
        return False

    # check if point is in simulation bounds
    def in_bounds(self, x, y, z):
        if x < self.xmin + radius or x > self.xmax - radius or y < self.ymin + radius or y > self.ymax - radius or z < self.zmin + radius or z > self.zmax - radius:
            return False
        return True


class RRT:
    def __init__(self):
        # starting node
        nstart = initial_position

        # tracks state of the object
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

    # check the distance between two nodes
    def distance_between(self, n1, n2):
        d = np.sqrt((self.state[0][n1] - self.state[0][n2]) ** 2 + (self.state[2][n1] - self.state[2][n2]) ** 2 + (
                self.state[4][n1] - self.state[4][n2]) ** 2)
        return d

    # expand a random node and test if its valid, connect to nearest node if it is
    def expand(self):
        x = np.zeros(12)
        in_obs = True

        #randomize a node until it is in a valid configuration
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

        # adds new node
        self.add_node(n, x)

        # checks nearest node and gets parameters of that node
        n_nearest = self.near(n)
        n_nearest = int(n_nearest)
        x_nearest = []
        for i in range(0, 12):
            x_nearest.append(self.state[i][n_nearest])
        nearest_parent = self.parent[n_nearest]
        t_nearest = self.time[nearest_parent]

        # steers the robot to the new node
        (x_new, u, col) = self.steer(x_nearest, x, t_nearest, t_nearest + dt)

        # removes the theoretical node
        self.remove_node(n)

        # if there was no sample free of collission, go to next loop
        if col is True:
            return

        # add the node with sampled dynamics
        else:
            self.add_node(n, x_new)
            self.add_edge(n_nearest, n)
            self.time.insert(n, t_nearest + dt)
            x_check = np.ma.array(x_new, mask=False)
            x_check.mask[6] = True
            x_check.mask[8] = True
            x_check.mask[10] = True
            self.dmax = np.linalg.norm(x_check.compressed() - goal)


    # find the nearest node
    def near(self, n):
        dmin = self.distance_between(0, n)
        nearest = 0
        for i in range(1, n):
            if self.distance_between(i, n) < dmin:
                dmin = self.distance_between(i, n)
                nearest = i
        return nearest

    # add node at position n with state x
    def add_node(self, n, x):
        for i in range(0, 12):
            self.state[i].insert(n, x[i])
    # remove node at position n
    def remove_node(self, n):
        for i in range(0, 12):
            self.state[i].pop(n)

    # connect two nodes
    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    # remove the edge between two nodes
    def remove_edge(self, n):
        self.parent.pop(n)

    # gets the number of nodes in the tree
    def number_of_nodes(self):
        return len(self.state[0])

    # steers the robot towards point x1 from point x0 by integrating dynamics over t0 to tf
    def steer(self, x0, x1, t0, tf):
        n_samples = 50 # points to sample
        u_candidates = []
        x_candidates = []
        col_list = []
        for i in range(0, n_samples):
            u_candidates.append([0, 0, 0, 0, 0, 0]) # assume nearest is starting node
            x_candidates.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            col_list.append(False)
        x_free = []
        u_free = []
        for i in range(0, n_samples):
            u_candidates[i] = self.sample_u() #randomly sample
            x_candidates[i], col_list[i] = self.propegate_dynamics(x0, u_candidates[i], t0, tf) # create path from sample
        for i in range(0, len(col_list)):

            # if no collision is found on the path, append it to available states
            if col_list[i] is False:
                x_free.append(x_candidates[i])
                u_free.append(u_candidates[i])

        if x_free == []:
            return None, None, True # if no free states, go to next loop
        else:
            # take path ending closest to desired node
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
        # random sampling
        u = np.zeros(6)
        u[0] = np.random.rand()
        u[1] = np.random.rand()
        u[2] = np.random.rand()
        u[3] = np.random.randn()
        u[4] = np.random.randn()
        u[5] = np.random.randn()
        return u

    def propegate_dynamics(self, x0, u, t0, tf):
        # create path by integrating dynamics
        def get_xdot(t, x):
            # dynamics from random sample
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
        sol = solve_ivp(get_xdot, [t0, tf], x0, t_eval=tsteps) # initial value problem
        xout = sol.y
        xnew = []

        # final point on path
        for i in range(0, 12):
            xnew.append(xout[i][-1])
        collision = False

        # check if any points along path lie in the obstacle
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
goal = np.array([10, 0, 0, 0, 0, 0, 0, 0, 0]) # desired final position [0,2,4], velocity[1,3,5], angular velocity[6,7,8]
eg = 0.25 # radius of goal region

# simulation boundaries
xmin = -5
xmax = 15
ymin = -10
zmin = -10
ymax = 10
zmax = 10

# initial step size
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
    #create graph
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d([xmin, xmax])
    ax.set_xlabel('X')
    ax.set_ylim3d([ymin, ymax])
    ax.set_ylabel('Y')
    ax.set_zlim3d([zmin, zmax])
    ax.set_zlabel('Z')

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

    print(goalstate)  # what loop was the goal found

    # draw path to goal if it exists
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
    draw(goalstate)


# run main when RRT is called
if __name__ == '__main__':
    main()
