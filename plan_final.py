from operator import add
import typing

# from scipy.spatial import distance
from scipy.spatial import ConvexHull
import numpy as np
from nest_info import Coordinate, DeliverySite, NestInfo

import math
import numpy as np
from sys import maxsize

import matplotlib.pyplot as plt


def get_path_length(path: typing.List["tuple"]) -> int:
    path_steps = np.diff(path, axis=0)
    return np.sum(np.linalg.norm(path_steps, axis=1))

show_animation = True
def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def closest(pt, others):
    return min(others, key = lambda i: distance(pt, i))


def even_select(N, M):
    if M > N/2:
        q, r = divmod(N, N-M)
        indices = [q*i + min(i, r) for i in range(N-M)]
    else:
        q, r = divmod(N, M)
        indices = [q*i + min(i, r) for i in range(M)]

    return indices

class State:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"  # tag for state
        self.h = 0
        self.k = 0

    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize

        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def set_state(self, state):
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state


class Map:

    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        elif k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(y, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []

        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            if show_animation:
                plt.plot(rx, ry, "-r")
                plt.pause(0.01)
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry

    def modify(self, state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break

def when_outside(goal, ox_1, oy_1, ox_2, oy_2, o1):

    path_array, out_pts_list= [], []
    start = [50, 26]
    temp_o1=o1
    temp_o1_ar=np.array(temp_o1)
    temp_hull = ConvexHull(temp_o1_ar)

    out_pts=temp_o1_ar[temp_hull.vertices]
    print("+++++")
    m = Map(100, 100)
    m.set_obstacle([(i, j) for i, j in o1])
    start = m.map[start[0]][start[1]]
    end = m.map[goal[0]][goal[1]]
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)
    for l in range(len(rx)):
        path_array.append([rx[l],ry[l]])

    path_array.append(goal)
     
    if get_path_length(path_array)>50:
        print(get_path_length(path_array))
        print("=====")
        out_pts_list=out_pts.tolist()
        new_o1 = [x for x in o1 if x not in out_pts_list]
        path_array=when_outside(goal, ox_1, oy_1, ox_2, oy_2, new_o1)
        return path_array
    else:
        print("here")
        return path_array
def when_inside(goal, ox_1, oy_1, ox_2, oy_2, temp_o1, out_pts):
    path_array, temp_path, out_pts_list=[], [], []
    # print(out_pts)
    start = [50, 26]
    # find shortest
    temp_goal=closest(goal,out_pts)                
    print(temp_goal)
    tup=[(i, j) for i, j in zip(ox_1, oy_1)]
    tup.remove((temp_goal[0], temp_goal[1]))
    m = Map(100, 100)
    m.set_obstacle(tup)
    # print(temp_goal)

    start = m.map[start[0]][start[1]]
    end = m.map[temp_goal[0]][temp_goal[1]]
    # print(start.x, start.y)
    # print(end.x, end.y)
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)

    for l in range(len(rx)):
        path_array.append([rx[l],ry[l]])

    m = Map(100, 100)
    m.set_obstacle([(i, j) for i, j in zip(ox_2, oy_2)])
    start_1 = temp_goal
    start = m.map[start_1[0]][start_1[1]]
    # print(start.x, start.y)
    end = m.map[goal[0]][goal[1]]
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)
    for l in range(len(rx)):
        temp_path.append([rx[l],ry[l]])
# outside outer set the outer as an obstacle

    temp_path.append(goal)
    total_dum=path_array+temp_path

    path_array=total_dum
    if get_path_length(total_dum)>50:
        outs_pts_list=out_pts.tolist()
        outs_pts_list.remove([temp_goal[0], temp_goal[1]])
        out_pts=np.array(outs_pts_list)
        path_array=when_inside(goal, ox_1, oy_1, ox_2, oy_2, temp_o1, out_pts)
        return path_array
    else:
        path_array=total_dum
        return path_array

    return path_array






class PathPlanner:
    def __init__(self, nest_info: NestInfo, delivery_sites: typing.List["DeliverySite"]):
        self.nest_info: NestInfo = nest_info
        self.delivery_sites: typing.List["DeliverySite"] = delivery_sites

    def plan_paths(self):
        o1, o2=[], []
        ox_1, oy_1, ox_2, oy_2 = [], [], [], []
        data = np.load('risk_zones.npy')

        for i in range(len(data)):
            for j in range(len(data[0])):
                if data[i][j]==2:
                    ox_2.append(i)
                    oy_2.append(j)
                    o2.append([i,j])
                elif data[i][j]==1:
                    ox_1.append(i)
                    oy_1.append(j)
                    o1.append([i,j])

        o1_ar=np.array(o1)
        hull = ConvexHull(o1_ar)
        
        for site in self.delivery_sites:
            print(site)
            path_array=[]
            path_array2=[]
            flag_1=0
            temp_path=[]
            flag_2=0
            total_dum=[]
            # for site in self.delivery_sites:

            m = Map(100, 100)

            start = [50, 26]
            goal = [site.coord.e, site.coord.n]
            print(goal)
            # inside outer 
            if goal in o1: 
                temp_o1=o1
                temp_o1_ar=np.array(temp_o1)
                # print(temp_o1)
                hull = ConvexHull(temp_o1_ar)
                out_pts=temp_o1_ar[hull.vertices]
                path_array=when_inside(goal, ox_1, oy_1, ox_2, oy_2, temp_o1, out_pts)
            else:
                if distance(start, goal)==50:
                    start = m.map[start[0]][start[1]]
                    end = m.map[goal[0]][goal[1]]
                    dstar = Dstar(m)
                else:
                    m.set_obstacle([(i, j) for i, j in zip(ox_1, oy_1)])
                    start = m.map[start[0]][start[1]]
                    end = m.map[goal[0]][goal[1]]
                    dstar = Dstar(m)
                rx, ry = dstar.run(start, end)
                for l in range(len(rx)):
                    path_array.append([rx[l],ry[l]])
                
                path_array.append(goal)
                 
                if get_path_length(path_array)>50:
                    print("=====")
                    # print(o1)
                    path_array=when_outside(goal, ox_1, oy_1, ox_2, oy_2, o1)
                




            idx = np.round(np.linspace(0, len(path_array) - 1, 10)).astype(int)
            for s in idx:
                path_array2.append(path_array[s])
            # path_array.append(goal)
            

            path_coords = [Coordinate(arr[0], arr[1]) for arr in path_array]
            
            path_steps = np.diff(path_coords, axis=0)
            # print(path_steps)
        # Once you have a solution for the site - populate it like this:
            print("--------") 
            path_length = get_path_length(path_array)
            print(path_length)
            print("-------")
            site.set_path(path_coords)


