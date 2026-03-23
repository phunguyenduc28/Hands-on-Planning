import numpy as np
import math
from matplotlib import pyplot as plt
import sys
import os 
from time import sleep
import datetime
from PIL import Image

import random

from online_motion_planning.Point import Point

class RRT():
    def __init__(self, delta_q, p, max_depth, min_dist):
        self.delta_q = delta_q # step
        self.p = p # random probability towards the goal
        self.max_depth = max_depth # max recursive depth for bisectional checking a segment free or not
        self.min_dist = min_dist # minimum distance to consider reach the goal

    def rand_conf(self, C, qgoal):
        """Random the configuration qrand in C with a bias to the qgoal \n
        Args:
        - C: robot configuration
        - p: bias probability of q_rand to be qgoal
        - qgoal: goal position"""
        if np.random.random() < self.p:
            return qgoal
        else:
            qrand = []
            conf_shape = C.shape[0]
            get_qrand = False 
            while(get_qrand == False):
                qrand = np.random.randint(0, conf_shape, 2)
                if C[qrand[0], qrand[1]] == 0:
                    get_qrand = True
            qrand = Point(qrand[0], qrand[1])
        return qrand

    def nearest_vertex(self, qrand, G):
        """Returns the vertice in graph G that has the smallest Eucledian distance to qrand \n
        Args:
        - qrand: random q
        - G: list of vertices in the tree"""
        qnear = G[0]
        nearest_dist = G[0].dist(qrand)
        for vertice in G:
            # q_x , q_y = vertice
            # dist = np.sqrt(np.square(q_x - qrand[0]) + np.square(q_y - qrand[1]))
            dist = vertice.dist(qrand)
            if dist < nearest_dist:
                qnear = vertice
                nearest_dist = dist
        return qnear

    def new_conf(self, qnear, qrand):
        """Returns a new configuration qnew by moving an incremental distance delta_q from qnear  in the direction of  qrand  without overshooting  qrand \n
        Agrs:
        - qnear: nearest q in G to qrand
        - qrand: random configuration q
        - delta_q: incremental distance"""
        if qnear.dist(qrand) < self.delta_q:
            return Point(qrand.x, qrand.y)
        vector = qnear.vector(qrand).unit()
        qnew = qnear.__add__(vector.scale(self.delta_q))
        return Point(int(qnew.x), int(qnew.y))

    def is_point_occupied(self, q, C):
        if q.x < 0 or q.x > C.shape[0] - 1 or q.y < 0 or q.y > C.shape[1] - 1:
            return True
        x = int(q.x)
        y = int(q.y)
        if q.x == 0 or q.x == C.shape[0] - 1 or q.y == 0 or q.y == C.shape[1] - 1:
            return C[x, y] == 1
        else:
            return C[x, y] == 1 or C[x+1, y] == 1 or C[x, y+1] == 1 or C[x+1, y+1] == 1 # round up error compensation
        
    def is_point_occupied_turtlebot(self, q, C):
        print("do something")

    def is_segment_free_bisection(self, qnear, qnew, C, depth):
        # Recursive function to check two-half segments free or occupied
        if qnear.x == qnew.x and  qnear.y == qnew.y:
            return False
        
        if self.is_point_occupied(qnear, C):
            return False
        if self.is_point_occupied(qnew, C):
            return False
        
        mid_point = qnear.__add__(qnew).__truediv__(2)
        if depth >= self.max_depth:
            return True
        
        if self.is_point_occupied(mid_point, C):
            return False
        
        left_segment = self.is_segment_free_bisection(qnear, mid_point, C, depth+1)
        if left_segment == False:
            return False
        right_segment = self.is_segment_free_bisection(mid_point, qnew, C, depth+1)
        return right_segment

    def sample(self, C, K, qstart_x, qstart_y, qgoal_x, qgoal_y):
        qstart = Point(qstart_x, qstart_y)
        qgoal = Point(qgoal_x, qgoal_y)
        G = [qstart]
        edges = []
        for k in range(K):
            qrand = self.rand_conf(C, qgoal)
            qnear = self.nearest_vertex(qrand, G)
            qnew = self.new_conf(qnear, qrand)
            if self.is_segment_free_bisection(qnear, qnew, C, 0):
                G.append(qnew)
                edges.append([G.index(qnear), G.index(qnew)])
                dist = qnew.dist(qgoal)
                if dist < self.min_dist:
                    print(f"Reach goal after {k} iterations")
                    G.append(qgoal)
                    edges.append([G.index(qnew), G.index(qgoal)])
                    return G, edges, k
        return G, edges, K

    def fill_path(self, vertices, edges):
        edges.reverse()
        path = [edges[0][1]]
        next_v = edges[0][0]
        i = 1
        while next_v != 0:
            while edges[i][1] != next_v:
                i += 1
            path.append(edges[i][1])
            next_v = edges[i][0]
            i = 1
        path.append(0)
        edges.reverse()
        path.reverse()
        return vertices, edges, path


    def smoothing(self, C, G, path):
        qfirst_idx = 0
        qlast_idx = len(path) - 1
        qfirst = G[path[qfirst_idx]]
        qlast = G[path[qlast_idx]]
        temp_path = []

        while qlast_idx > 0:
            dist = qfirst.dist(qlast)
            self.max_depth = round(math.log(dist, 2)) + 1
            while self.is_segment_free_bisection(qfirst, qlast, C, 0) is not True:
                if qfirst_idx + 1 == qlast_idx:
                    break
                qfirst_idx += 1
                qfirst = G[path[qfirst_idx]]
            # print(qfirst_idx, qlast_idx)
            temp_path = temp_path + [path[qlast_idx]]
            qlast_idx = qfirst_idx
            qlast = G[path[qlast_idx]]
            qfirst_idx = 0
            qfirst = G[path[qfirst_idx]]
        
        temp_path = temp_path + [path[qfirst_idx]]
        smooth_path = temp_path[::-1] # reverse the path
        return smooth_path

    def calculate_distace_start_to_goal(self, path, G):
        total_dist = 0
        if len(path) == 2: # short distance travel
            current_vertice = G[path[0]]
            next_vertice = G[path[1]]
            total_dist += current_vertice.dist(next_vertice)
        else:
            for i in range(0, len(path) - 2): 
                current_vertice = G[path[i]]
                next_vertice = G[path[i+1]]
                total_dist += current_vertice.dist(next_vertice)
        return total_dist
    
    def plot(self, grid_map, states, edges, path):
        fig, ax = plt.subplots(figsize=(10, 10))

        cax = ax.matshow(grid_map)

        for i, v in enumerate(states):
            ax.plot(v.y, v.x, "+w")

        # Plot edges
        for e in edges:
            ax.plot(
                [states[e[0]].y, states[e[1]].y],
                [states[e[0]].x, states[e[1]].x],
                "--g",
            )

        for i in range(1, len(path)):
            ax.plot(
                [states[path[i - 1]].y, states[path[i]].y],
                [states[path[i - 1]].x, states[path[i]].x],
                "r",
            )

        ax.plot(states[0].y, states[0].x, "r*")
        ax.plot(states[-1].y, states[-1].x, "b*")

        fig.colorbar(cax)
        # plt.tight_layout()
        # plt.show()
        plt.savefig('map.png', dpi=400)

if __name__ == "__main__": 
    # Read input from command line
    image_path = str(sys.argv[1]) # file path
    K = int(sys.argv[2]) # number of iterations
    delta_q = int(sys.argv[3]) # step
    p = float(sys.argv[4]) # probability q_rand to be q_goal
    qstart_x = int(sys.argv[5]) # qstart coordinates
    qstart_y = int(sys.argv[6])
    qgoal_x = int(sys.argv[7]) # qgoal coordinates
    qgoal_y = int(sys.argv[8])

    # Initilization
    max_depth = round(math.log(delta_q, 2)) + 1 # max iteration for considering a segment free or not
    min_dist = 5

    # Load grid map
    image = Image.open(image_path).convert("L")
    grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1]) / 255
    # binarize the image
    grid_map[grid_map > 0.5] = 1
    grid_map[grid_map <= 0.5] = 0
    # Invert colors to make 0 -> free and 1 -> occupied
    grid_map = (grid_map * -1) + 1
    # Show grid map
    fig, ax = plt.subplots(figsize=(10, 10))
    cax = ax.matshow(grid_map)
    fig.colorbar(cax)
    plt.show()


    path = []
    rrt = RRT(delta_q, p, max_depth, min_dist)
    G, edges, iter = rrt.sample(grid_map, K, qstart_x, qstart_y, qgoal_x, qgoal_y)
    if iter == K:
        print(f"Cannot find a path within a defined {iter} iteration . Try increase it!")
    else:
        G, edges, path = rrt.fill_path(G, edges)
        path = rrt.smoothing(grid_map, G, path)
        dist_to_goal = rrt.calculate_distace_start_to_goal(path, G)
        print(f"Distance from start to goal {dist_to_goal}")
        rrt.plot(grid_map, G, edges, path)
    