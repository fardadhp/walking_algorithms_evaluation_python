import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
import random
from tkinter import *
from shapely.geometry.polygon import Polygon
from descartes import PolygonPatch
from matplotlib.path import Path
from matplotlib.patches import Circle
from shapely.geometry import LineString
import wx


class Agent:
    def __init__(self, ID, xcor, ycor, destination):
        global algorithm
        self.ID = ID
        self.xcor = xcor
        self.ycor = ycor
        self.wspeed = 1
        self.destination = destination
        self.follow_wall = False
        self.h = 'nobody'
        self.heading = 0
        self.d_min = 0
        self.r = size
        self.step = self.r
        self.br_dist = 0
        self.int_target = 'nobody'
        self.mline = []
        self.pre_node = [self.xcor, self.ycor]
        self.b_node = 'nobody'
        self.b_vertices = []
        self.n_vertex = 0
        self.b_inc = 0
        self.obs_num = None
        if random.randint(0, 1) == 0:
            self.direction = 1
        else:
            self.direction = -1
        self.face_target(self.destination)
        if algorithm == 'Bug2':
            self.create_mline()
        if algorithm == 'DistBug':
            self.cn = 0
            self.ext2 = False
            self.ext3 = False
            self.v_obs = []
        if algorithm == 'Bug1':
            self.l = 'nobody'
            self.search_mode = False
            self.r2 = 0
            self.r3 = 0
        if algorithm == 'KBug':
            self.step_forward = False
            self.node = 'nobody'
            if not self.target_visible(self.destination):
                self.h = self.find_wall(self.heading,self.r)
            else:
                self.h = 'nobody'
            self.last_node = 'nobody'
            if self.h != 'nobody':
                self.find_node()
                self.face_target(self.node)
        if algorithm == 'TangentBug':
            self.ltg_list = []
            self.transition1 = False
            self.transition2 = False

    def wall(self, angle, step):
        wall = 'nobody'
        x2 = self.xcor + np.cos(angle) * step
        y2 = self.ycor + np.sin(angle) * step
        if not all([(a > b) for a, b in zip([size, size], [x2, y2])]):
            wall = 'nobody'
        else:
            n = 0
            check = False
            while check is False and n < len(obs_path):
                if obs_path[n].contains_point([x2, y2]):
                    wall = [x2, y2]
                    check = True
                    self.obs_num = n
                else:
                    n += 1
        return wall

    def face_target(self, target):
        if (target[0] - self.xcor) == 0:
            if (target[1] - self.ycor) > 0:
                self.heading = np.pi/2
            else:
                self.heading = -np.pi/2
        else:
            self.heading = np.arctan((target[1] - self.ycor) / (target[0] - self.xcor))
            if (target[0] - self.xcor) < 0:
                self.heading = self.heading + np.pi
        self.turn_positive(0)

    def fd(self, step):
        x2 = self.xcor + step*np.cos(self.heading)
        y2 = self.ycor + step*np.sin(self.heading)
        self.xcor = x2
        self.ycor = y2

    def target_visible(self, target):
        output = True
        if distance2v([self.xcor, self.ycor], target) <= self.r:
            path = Path([(self.xcor, self.ycor), target])
            n = 0
            while output is True and n < len(obs_path):
                if path.intersects_path(obs_path[n]):
                    output = False
                else:
                    n += 1
        else:
            output = False
        return output

    def turn_positive(self, angle):
        self.heading = self.heading + angle
        if self.heading < 0:
            self.heading = 2*np.pi + self.heading
        elif self.heading >= 2*np.pi:
            self.heading = self.heading - 2*np.pi

    def create_mline(self):
        nodes = [(self.xcor, self.ycor), destination]
        self.mline = Path(nodes)

    def build_v_obs(self, d):
        circle = Circle(destination, d)
        path = circle.get_path()
        transform = circle.get_transform()
        self.v_obs = transform.transform_path(path)

    def find_wall(self, direct, dist):
        output = 'nobody'
        x2 = self.xcor + dist * np.cos(direct)
        y2 = self.ycor + dist * np.sin(direct)
        line1 = LineString([(self.xcor, self.ycor), (x2, y2)])
        path = Path([(self.xcor, self.ycor), (x2, y2)])
        n = -1
        check = False
        while n < len(obs_path) and check is False:
            n += 1
            check = path.intersects_path(obs_path[n])
        self.obs_num = n
        nodes = bound_list[n]
        nodes.append(nodes[0])
        line2 = LineString(nodes)
        intersection = line1.intersection(line2)
        if abs(intersection[0].x - self.xcor) < abs(intersection[-1].x - self.xcor):
            return [intersection[0].x, intersection[0].y]
        else:
            return [intersection[-1].x, intersection[-1].y]

    def find_node(self):
        tmp = self.heading
        self.face_target(self.destination)
        tmp2 = self.heading
        self.h = self.find_wall(self.heading, self.r)
        self.heading = tmp
        x2 = (self.xcor + (distance2v([self.xcor, self.ycor], self.h) - 1) * np.cos(tmp2)) // 1
        y2 = (self.ycor + (distance2v([self.xcor, self.ycor], self.h) - 1) * np.sin(tmp2)) // 1
        hh = [x2 + 0.5, y2 + 0.5]
        lst = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if patch_list[int((x2 + i) + (y2 + j) * size)][2] != (0.0, 0.0, 0.0, 1.0) and [self.xcor // 1,
                                                                                               self.ycor // 1] != [
                            x2 + i, y2 + j] and self.target_visible([hh[0] + i, hh[1] + j]) == True:
                    nbrs = []
                    for s in [-1, 0, 1]:
                        for t in [-1, 0, 1]:
                            nbrs.append([x2 + i + s, y2 + j + t])
                    k = 0
                    ch = False
                    while k < len(nbrs) and ch is False:
                        if patch_list[int((nbrs[k][0]) + (nbrs[k][1]) * size)][2] == (0.0, 0.0, 0.0, 1.0):
                            ch = True
                        else:
                            k = k + 1
                    if ch is True:
                        lst.append([x2 + i, y2 + j])
        lyr = lst
        while lyr != []:
            ph = [self.xcor // 1, self.ycor // 1]
            a = []
            for f in range(len(lyr)):
                for m in [-1, 0, 1]:
                    for n in [-1, 0, 1]: 
                        nbrs = []
                        for m1 in [-1, 0, 1]:
                            for n1 in [-1, 0, 1]:
                                nbrs.append([lyr[f][0] + m + m1, lyr[f][1] + n + n1])
                        k = 0
                        ch = False
                        while k < len(nbrs) and ch is False:
                            if patch_list[int((nbrs[k][0]) + (nbrs[k][1]) * size)][2] == (0.0, 0.0, 0.0, 1.0):
                                ch = True
                            else:
                                k = k + 1
                        if ch is True:
                            if [lyr[f][0] + m, lyr[f][1] + n] != ph and \
                                            patch_list[int((lyr[f][0] + m) + (lyr[f][1] + n) * size)][2] != (
                                            0.0, 0.0, 0.0, 1.0) and not [lyr[f][0] + m, lyr[f][1] + n] in lst:
                                a.append([lyr[f][0] + m, lyr[f][1] + n])

            if a != []:
                a2 = []
                for i in range(len(a)):
                    if self.target_visible(a[i]):
                        a2.append(a[i])
            lyr = a2
            if lyr != []:
                lyr = np.unique(lyr, axis=0)
                lyr = lyr.tolist()
            lst = lst + lyr
            lst = np.unique(lst, axis=0)
            lst = lst.tolist()

        edge = lst
        head = tmp2
        rs = []
        for i in range(len(edge)):
            self.face_target(edge[i])
            if (self.heading - head) % (2 * np.pi) < np.pi:
                rs.append(edge[i])
        dist = []
        for i in range(len(rs)):
            dist.append(distance2v([self.xcor, self.ycor], rs[i]))
        dist2 = sorted(dist)
        rs2 = []
        for i in range(len(rs)):
            rs2.append(rs[dist.index(dist2[i])])

        a = 0
        node1 = 'nobody'
        for i in range(len(rs2)):
            self.face_target(rs2[i])
            b = (self.heading - head) % (2 * np.pi)
            if b >= a:
                node1 = rs2[i]
                a = b
            elif abs(b - a) < 10 / 180 * np.pi and distance2v([self.xcor, self.ycor], rs2[i]) > distance2v(
                    [self.xcor, self.ycor], node1):
                node1 = rs2[i]
                a = b

        ls = []
        for i in range(len(edge)):
            self.face_target(edge[i])
            if (self.heading - head) % (2 * np.pi) >= np.pi:
                ls.append(edge[i])
        dist = []
        for i in range(len(ls)):
            dist.append(distance2v([self.xcor, self.ycor], ls[i]))
        dist2 = sorted(dist)
        ls2 = []
        for i in range(len(ls)):
            ls2.append(ls[dist.index(dist2[i])])

        a = 2 * np.pi
        node2 = 'nobody'
        for i in range(len(ls2)):
            self.face_target(ls2[i])
            b = (self.heading - head) % (2 * np.pi)
            if b <= a:
                node2 = ls2[i]
                a = b
            elif abs(b - a) < 10 / 180 * np.pi and distance2v([self.xcor, self.ycor], ls2[i]) > distance2v(
                    [self.xcor, self.ycor], node2):
                node2 = ls2[i]
                a = b

        if node1 == 'nobody':
            self.node = node2
        elif node2 == 'nobody':
            self.node = node1
        elif distance2v([self.xcor // 1, self.ycor // 1], node1) < distance2v([self.xcor // 1, self.ycor // 1], node2):
            self.node = node1
        elif distance2v([self.xcor // 1, self.ycor // 1], node1) == distance2v([self.xcor // 1, self.ycor // 1], node2):
            if distance2v(self.destination, node1) < distance2v(self.destination, node2):
                self.node = node1
            else:
                self.node = node2
        else:
            self.node = node2

        if self.node == self.last_node:
            if self.node == node1:
                self.node = node2
            else:
                self.node = node1

    def walk_cope0(self):
        global size, n_agents_left, active_agents, n_trapped
        if self.follow_wall:
            if distance2v(self.b_node, [self.xcor, self.ycor]) < self.wspeed / 2:
                if distance2v([self.xcor, self.ycor], self.destination) > self.br_dist:
                    self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                    self.b_node = self.b_vertices[self.n_vertex]
                    self.face_target(self.b_node)
                else:
                    self.follow_wall = False
                    self.face_target(self.destination)
        next_step = self.wall(self.heading, self.wspeed)
        if next_step != 'nobody':
            if self.follow_wall:
                self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
            else:
                self.follow_wall = True
                self.h = [self.xcor, self.ycor]
                self.d_min = distance2v([self.xcor, self.ycor], self.destination)
                self.create_mline()
                self.b_vertices = bound_list[self.obs_num]
                x1 = self.xcor - 3 * np.cos(self.heading)
                y1 = self.ycor - 3 * np.sin(self.heading)
                x2 = self.xcor + np.cos(self.heading)
                y2 = self.ycor + np.sin(self.heading)
                line1 = LineString([(x1, y1), (x2, y2)])
                check = False
                n = 0
                bounds = self.b_vertices.copy()
                bounds.append(bounds[0])
                while n < (len(bounds) - 1) and check is False:
                    line2 = LineString([bounds[n], bounds[n + 1]])
                    intersection = line1.intersection(line2)
                    if not intersection.is_empty:
                        check = True
                        self.h = [self.xcor, self.ycor]
                        self.l = self.h
                    else:
                        n += 1
                nodes = self.b_vertices.copy()
                nodes.append(nodes[0])
                line2 = LineString(nodes)
                intersection = LineString(self.mline.vertices).intersection(line2)
                if intersection.geom_type == 'Point':
                    self.int_target = [intersection.x, intersection.y]
                else:
                    if abs(intersection[0].x - self.xcor) < abs(intersection[-1].x - self.xcor):
                        self.int_target = [intersection[-1].x, intersection[-1].y]
                    else:
                        self.int_target = [intersection[0].x, intersection[0].y]
                self.br_dist = distance2v(self.int_target, self.destination)
# ##### extension 1
                node1 = [n % len(self.b_vertices), self.b_vertices[n % len(self.b_vertices)]]
                node2 = [(n+1) % len(self.b_vertices), self.b_vertices[(n+1) % len(self.b_vertices)]]
                if node1[1][0] == node2[1][0]:
                    theta = np.pi/2
                    if node1[1][1] > node2[1][1]:
                        n1 = node2
                        n2 = node1
                    else:
                        n1 = node1
                        n2 = node2
                else:
                    theta = np.arctan((node2[1][1]-node1[1][1])/(node2[1][0]-node1[1][0]))
                    if node1[1][0] < node2[1][0]:
                        n1 = node1
                        n2 = node2
                    else:
                        n1 = node2
                        n2 = node1
                if 0 <= self.heading <= np.pi/2:
                    if theta < self.heading - np.pi/2:
                        self.n_vertex = n1[0]
                    else:
                        self.n_vertex = n2[0]
                elif self.heading <= np.pi:
                    if theta > self.heading - np.pi/2:
                        self.n_vertex = n2[0]
                    else:
                        self.n_vertex = n1[0]
                elif self.heading <= 3*np.pi/2:
                    if theta < self.heading - 3*np.pi/2:
                        self.n_vertex = n2[0]
                    else:
                        self.n_vertex = n1[0]
                else:
                    if theta > self.heading - 3*np.pi/2:
                        self.n_vertex = n1[0]
                    else:
                        self.n_vertex = n2[0]

                if self.n_vertex == n:
                    self.b_inc = -1
                else:
                    self.b_inc = 1
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
                if distance2v([self.xcor, self.ycor], self.b_node) < self.wspeed / 2:
                    self.xcor = self.b_node[0]
                    self.ycor = self.b_node[1]
                    self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                    self.b_node = self.b_vertices[self.n_vertex]
                    self.face_target(self.b_node)
                next_step = self.wall(self.heading, self.wspeed)
                if next_step != 'nobody':
                    self.xcor = self.b_vertices[(self.n_vertex - self.b_inc) % len(self.b_vertices)][0]
                    self.ycor = self.b_vertices[(self.n_vertex - self.b_inc) % len(self.b_vertices)][1]
                    self.face_target(self.b_node)
# ##########
        if distance2v([self.xcor, self.ycor], self.destination) <= 1:
            self.xcor = self.destination[0]
            self.ycor = self.destination[1]
            n_agents_left += -1
            active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
            del self
        else:
            # people_ahead = {add code later}
            self.fd(self.wspeed)
            if self.follow_wall:
                if distance2v([self.xcor, self.ycor], self.destination) < self.d_min:
                    self.d_min = distance2v([self.xcor, self.ycor], self.destination)
                if self.target_visible(self.destination) or distance2v([self.xcor, self.ycor], self.destination) - self.r <self.d_min - self.step or distance2v([self.xcor, self.ycor], self.int_target) < self.wspeed/2:
                    self.follow_wall = False
                    self.face_target(self.destination)
                    self.d_min = 0
                    self.h = 'nobody'
                x2 = self.xcor + self.wspeed*np.cos(self.heading + np.pi)
                y2 = self.ycor + self.wspeed*np.sin(self.heading + np.pi)
                if self.h != 'nobody' and distance2v([self.xcor, self.ycor], self.h) < 1.5*self.wspeed/2 and distance2v([x2, y2], self.h) > self.wspeed/2:
                    active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                    del self
                    n_trapped += 1
                    n_agents_left -= 1
                elif distance2v([self.xcor, self.ycor], self.destination) < self.wspeed:
                    active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                    del self
                    n_agents_left += -1

    def walk_bug1(self):
        global size, n_agents_left, active_agents, n_trapped
        if self.follow_wall:
            if distance2v([self.xcor, self.ycor], self.b_node) < self.wspeed/2:
                self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
        next_step = self.wall(self.heading, self.wspeed)
        if next_step != 'nobody':
            if self.follow_wall:
                self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
            else:
                self.search_mode = True
                self.follow_wall = True
                self.q = distance2v([self.xcor, self.ycor], self.destination)
                self.b_vertices = bound_list[self.obs_num]
                x1 = self.xcor - 3 * np.cos(self.heading)
                y1 = self.ycor - 3 * np.sin(self.heading)
                x2 = self.xcor + np.cos(self.heading)
                y2 = self.ycor + np.sin(self.heading)
                line1 = LineString([(x1, y1), (x2, y2)])
                check = False
                n = 0
                bounds = self.b_vertices.copy()
                bounds.append(bounds[0])
                while n < (len(bounds) - 1) and check is False:
                    line2 = LineString([bounds[n], bounds[n + 1]])
                    intersection = line1.intersection(line2)
                    if not intersection.is_empty:
                        check = True
                        self.h = [self.xcor, self.ycor]
                        self.l = self.h
                    else:
                        n += 1
                self.n_vertex = (n + random.randint(0, 1)) % len(self.b_vertices)
                if self.n_vertex == n:
                    self.b_inc = -1
                else:
                    self.b_inc = 1
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
                if distance2v([self.xcor, self.ycor], self.b_node) < self.wspeed/2:
                    self.xcor = self.b_node[0]
                    self.ycor = self.b_node[1]
                    self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                    self.b_node = self.b_vertices[self.n_vertex]
                    self.face_target(self.b_node)
                next_step = self.wall(self.heading, self.wspeed)
                if next_step != 'nobody':
                    self.xcor = self.b_vertices[(self.n_vertex - self.b_inc) % len(self.b_vertices)][0]
                    self.ycor = self.b_vertices[(self.n_vertex - self.b_inc) % len(self.b_vertices)][1]
                    self.face_target(self.b_node)
        if distance2v([self.xcor, self.ycor], self.destination) <= self.wspeed:
            self.xcor = self.destination[0]
            self.ycor = self.destination[1]
            n_agents_left += -1
            active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
            del self
        else:
            # people_ahead = {add code later}
            self.fd(self.wspeed)
            if self.search_mode:
                self.r2 = self.r2 + self.wspeed
                self.r3 = self.r3 + self.wspeed
                dist = distance2v([self.xcor, self.ycor], self.destination)
                if dist < self.q:
                    self.q = dist
                    self.l = [self.xcor, self.ycor]
                    self.r3 = 0
                x2 = self.xcor + self.wspeed * np.cos(self.heading + np.pi)
                y2 = self.ycor + self.wspeed * np.sin(self.heading + np.pi)
                if self.h != 'nobody':
                    if distance2v([self.xcor, self.ycor], self.h) < 1.5*self.wspeed/2 and distance2v([x2, y2], self.h) > self.wspeed/2 and self.l != 'nobody':
                        self.search_mode = False
                        if self.r2 / 2 > self.r3:
                            self.turn_positive(np.pi)
                            self.direction = -self.direction
                            self.b_inc = -self.b_inc
                            self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                            self.b_node = self.b_vertices[self.n_vertex]
                            self.face_target(self.b_node)
            else:
                if self.l != 'nobody' and distance2v([self.xcor, self.ycor], self.l) < 1.5*self.wspeed/2:
                    self.follow_wall = False
                    self.search_mode = False
                    self.face_target(self.destination)
                    self.r2 = 0
                    self.r3 = 0
                    self.q = 0
                    self.h = 'nobody'
                    self.l = 'nobody'
                    next_step = self.wall(self.heading, self.wspeed)
                    if next_step != 'nobody':
                        active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                        del self
                        n_trapped += 1
                        n_agents_left -= 1
                    elif self.ID in active_agents and distance2v([self.xcor, self.ycor], self.destination) < self.wspeed:
                        active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                        del self
                        n_agents_left += -1

    def walk_bug2(self):
        global size, n_agents_left, active_agents, n_trapped
        if self.follow_wall:
            if distance2v(self.b_node, [self.xcor, self.ycor]) < self.wspeed / 2:
                self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
        next_step = self.wall(self.heading, self.wspeed)
        if next_step != 'nobody':
            if self.follow_wall:
                self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
            else:
                self.follow_wall = True
                self.h = [self.xcor, self.ycor]
                self.b_vertices = bound_list[self.obs_num]
                x1 = self.xcor - 3 * np.cos(self.heading)
                y1 = self.ycor - 3 * np.sin(self.heading)
                x2 = self.xcor + np.cos(self.heading)
                y2 = self.ycor + np.sin(self.heading)
                line1 = LineString([(x1, y1), (x2, y2)])
                check = False
                n = 0
                bounds = self.b_vertices.copy()
                bounds.append(bounds[0])
                while n < (len(bounds) - 1) and check is False:
                    line2 = LineString([bounds[n], bounds[n + 1]])
                    intersection = line1.intersection(line2)
                    if not intersection.is_empty:
                        check = True
                        self.h = [self.xcor, self.ycor]
                    else:
                        n += 1
                self.n_vertex = (n + random.randint(0, 1)) % len(self.b_vertices)
                if self.n_vertex == n:
                    self.b_inc = -1
                else:
                    self.b_inc = 1
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
                if distance2v([self.xcor, self.ycor], self.b_node) < self.wspeed/2:
                    self.xcor = self.b_node[0]
                    self.ycor = self.b_node[1]
                    self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                    self.b_node = self.b_vertices[self.n_vertex]
                    self.face_target(self.b_node)
                next_step = self.wall(self.heading, self.wspeed)
                if next_step != 'nobody':
                    self.xcor = self.b_vertices[(self.n_vertex - self.b_inc) % len(self.b_vertices)][0]
                    self.ycor = self.b_vertices[(self.n_vertex - self.b_inc) % len(self.b_vertices)][1]
                    self.face_target(self.b_node)
        if distance2v([self.xcor, self.ycor], self.destination) <= 1:
            self.xcor = self.destination[0]
            self.ycor = self.destination[1]
            n_agents_left += -1
            active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
            del self
        else:
            # people_ahead = {add code later}
            self.fd(self.wspeed)
            if self.follow_wall:
                x1 = (self.xcor - np.cos(self.heading))
                y1 = (self.ycor - np.sin(self.heading))
                x2 = (self.xcor + np.cos(self.heading))
                y2 = (self.ycor + np.sin(self.heading))
                line1 = Path([(self.xcor, self.ycor), (x1, y1)])
                if line1.intersects_path(self.mline) and distance2v([self.xcor, self.ycor], self.h) > self.wspeed/2 and distance2v([x2, y2], self.h) > self.wspeed/2 and distance2v([self.xcor, self.ycor], self.destination) < distance2v(self.h, self.destination):
                    self.follow_wall = False
                    self.face_target(self.destination)
                    self.h = 'nobody'
                x2 = self.xcor + self.wspeed*np.cos(self.heading + np.pi)
                y2 = self.ycor + self.wspeed*np.sin(self.heading + np.pi)
                if self.h != 'nobody' and distance2v([self.xcor, self.ycor], self.h) < 1.5*self.wspeed/2 and distance2v([x2, y2], self.h) > self.wspeed/2:
                    active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                    del self
                    n_trapped += 1
                    n_agents_left -= 1
                elif distance2v([self.xcor, self.ycor], self.destination) < self.wspeed:
                    active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                    del self
                    n_agents_left += -1

    def walk_distbug(self):
        global size, n_agents_left, active_agents, n_trapped
        if self.follow_wall:
            # extension 2
            if extension2:
                if not self.ext2:
                    angle = 0
                    if (self.destination[0] - self.xcor) == 0:
                        if (self.destination[1] - self.ycor) > 0:
                            angle = np.pi / 2
                        else:
                            angle = -np.pi / 2
                    else:
                        angle = np.arctan((self.destination[1] - self.ycor) / (self.destination[0] - self.xcor))
                        if (self.destination[0] - self.xcor) < 0:
                            angle = angle + np.pi
                    if angle < 0:
                        angle = 2 * np.pi + angle
                    elif angle >= 2 * np.pi:
                        angle = angle - 2 * np.pi
                    if np.abs(angle) > np.pi:
                        angle = 2 * np.pi - angle
                    if angle > 135 / 180 * np.pi:
                        self.direction = -self.direction
                        self.turn_positive(np.pi)
                        self.ext2 = True
            ###########
            if distance2v(self.b_node, [self.xcor, self.ycor]) < self.wspeed / 2:
                self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
        next_step = self.wall(self.heading, self.wspeed)
        if next_step != 'nobody':
            if self.follow_wall:
                self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
            else:
                self.follow_wall = True
                self.h = [self.xcor, self.ycor]
                self.create_mline()
                self.d_min = distance2v([self.xcor, self.ycor], self.destination)
                self.b_vertices = bound_list[self.obs_num]
                x1 = self.xcor - 3 * np.cos(self.heading)
                y1 = self.ycor - 3 * np.sin(self.heading)
                x2 = self.xcor + np.cos(self.heading)
                y2 = self.ycor + np.sin(self.heading)
                line1 = LineString([(x1, y1), (x2, y2)])
                check = False
                n = 0
                bounds = self.b_vertices.copy()
                bounds.append(bounds[0])
                while n < (len(bounds) - 1) and check is False:
                    line2 = LineString([bounds[n], bounds[n + 1]])
                    intersection = line1.intersection(line2)
                    if not intersection.is_empty:
                        check = True
                        self.h = [self.xcor, self.ycor]
                        self.l = self.h
                    else:
                        n += 1
                nodes = self.b_vertices.copy()
                nodes.append(nodes[0])
                line2 = LineString(nodes)
                intersection = LineString(self.mline.vertices).intersection(line2)
                if intersection.geom_type == 'Point':
                    self.int_target = [intersection.x, intersection.y]
                else:
                    if abs(intersection[0].x - self.xcor) < abs(intersection[-1].x - self.xcor):
                        self.int_target = [intersection[-1].x, intersection[-1].y]
                    else:
                        self.int_target = [intersection[0].x, intersection[0].y]
                self.br_dist = distance2v(self.int_target, self.destination)
                if extension3:
                    self.build_v_obs(distance2v([self.xcor, self.ycor], self.destination))
                # extension 1
                node1 = [n % len(self.b_vertices), self.b_vertices[n % len(self.b_vertices)]]
                node2 = [(n+1) % len(self.b_vertices), self.b_vertices[(n+1) % len(self.b_vertices)]]
                if node1[1][0] == node2[1][0]:
                    theta = np.pi/2
                    if node1[1][1] > node2[1][1]:
                        n1 = node2
                        n2 = node1
                    else:
                        n1 = node1
                        n2 = node2
                else:
                    theta = np.arctan((node2[1][1]-node1[1][1])/(node2[1][0]-node1[1][0]))
                    if node1[1][0] < node2[1][0]:
                        n1 = node1
                        n2 = node2
                    else:
                        n1 = node2
                        n2 = node1
                if 0 <= self.heading <= np.pi/2:
                    if theta < self.heading - np.pi/2:
                        self.n_vertex = n1[0]
                    else:
                        self.n_vertex = n2[0]
                elif self.heading <= np.pi:
                    if theta > self.heading - np.pi/2:
                        self.n_vertex = n2[0]
                    else:
                        self.n_vertex = n1[0]
                elif self.heading <= 3*np.pi/2:
                    if theta < self.heading - 3*np.pi/2:
                        self.n_vertex = n2[0]
                    else:
                        self.n_vertex = n1[0]
                else:
                    if theta > self.heading - 3*np.pi/2:
                        self.n_vertex = n1[0]
                    else:
                        self.n_vertex = n2[0]

                if self.n_vertex == n:
                    self.b_inc = -1
                else:
                    self.b_inc = 1
                self.b_node = self.b_vertices[self.n_vertex]
                self.face_target(self.b_node)
                if distance2v([self.xcor, self.ycor], self.b_node) < self.wspeed / 2:
                    self.xcor = self.b_node[0]
                    self.ycor = self.b_node[1]
                    self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                    self.b_node = self.b_vertices[self.n_vertex]
                    self.face_target(self.b_node)
                if next_step != 'nobody':
                    self.xcor = self.b_vertices[(self.n_vertex - self.b_inc) % len(self.b_vertices)][0]
                    self.ycor = self.b_vertices[(self.n_vertex - self.b_inc) % len(self.b_vertices)][1]
                    self.face_target(self.b_node)
                ##########
        if distance2v([self.xcor, self.ycor], self.destination) <= 1:
            self.xcor = self.destination[0]
            self.ycor = self.destination[1]
            n_agents_left += -1
            active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
            del self
        else:
            # people_ahead = {add code later}
            self.fd(self.wspeed)
            if self.follow_wall:
                if extension3:
                    x2 = self.xcor + np.cos(self.heading)
                    y2 = self.ycor + np.sin(self.heading)
                    line1 = Path([(self.xcor, self.ycor), (x2, y2)])
                    if line1.intersects_path(self.v_obs):
                        if self.ext3:
                            self.build_v_obs(distance2v([self.xcor, self.ycor], self.destination) + self.r / 2)
                            self.ext3 = False
                        else:
                            self.direction = -self.direction
                            self.turn_positive(np.pi)
                            self.ext3 = True
                            self.b_inc = -self.b_inc
                            self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                            self.b_node = self.b_vertices[self.n_vertex]
                            self.face_target(self.b_node)
                            if distance2v(self.b_node, [self.xcor, self.ycor]) < self.wspeed / 2:
                                self.n_vertex = (self.n_vertex + self.b_inc) % len(self.b_vertices)
                                self.b_node = self.b_vertices[self.n_vertex]
                                self.face_target(self.b_node)

                if distance2v([self.xcor, self.ycor], self.destination) < self.d_min:
                    self.d_min = distance2v([self.xcor, self.ycor], self.destination)
                x1 = (self.xcor - np.cos(self.heading))
                y1 = (self.ycor - np.sin(self.heading))
                line1 = Path([(self.xcor, self.ycor), (x1, y1)])
                if self.target_visible(self.destination) or distance2v([self.xcor, self.ycor], self.destination) - self.r < self.d_min - self.step or (line1.intersects_path(self.mline) and distance2v([self.xcor, self.ycor], self.h) > self.wspeed and distance2v([self.xcor, self.ycor], self.destination) < distance2v(self.h, self.destination)):
                    self.follow_wall = False
                    self.face_target(self.destination)
                    self.d_min = 0
                    self.h = 'nobody'
                    self.mline = []
                    self.ext2 = False
                    self.cn = 0
                    self.ext3 = False
                    self.v_obs = []
                x2 = self.xcor + self.wspeed * np.cos(self.heading + np.pi)
                y2 = self.ycor + self.wspeed * np.sin(self.heading + np.pi)
                if self.h != 'nobody' and distance2v([self.xcor, self.ycor],
                                                     self.h) < 1.5 * self.wspeed / 2 and distance2v([x2, y2],
                                                                                                    self.h) > self.wspeed / 2:
                    if self.cn == 1 or (not self.ext3 and extension3 and not extension2) or (not extension3 and not extension2):
                        active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                        del self
                        n_trapped += 1
                        n_agents_left -= 1
                    else:
                        self.cn = 1
                elif distance2v([self.xcor, self.ycor], self.destination) < self.wspeed:
                    active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                    del self
                    n_agents_left += -1

    def walk_kbug(self):
        global size, n_agents_left, active_agents, n_trapped
        self.step_forward = False
        tmp = self.heading
        self.face_target(self.destination)
        tmp2 = self.heading
        if self.find_wall(self.heading, self.r) != 'nobody' and self.node == 'nobody':
            self.node = [self.xcor // 1, self.ycor // 1]
        self.heading = tmp
        if [self.xcor // 1, self.ycor // 1] != self.node:
            self.step_forward = True
        else:
            self.node = 'nobody'
            if self.target_visible(self.destination) == True or self.find_wall(tmp2, self.r) == 'nobody':
                self.face_target(self.destination)
                self.step_forward = True
            else:
                self.find_node()
                self.last_node = [self.xcor // 1, self.ycor // 1]
                self.face_target(self.node)
                self.step_forward = True

        if self.step_forward == True:
            # add human interaction
            self.fd(self.wspeed)
        if [self.xcor // 1 + 0.5, self.ycor // 1 + 0.5] == self.destination:
            active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
            del self
            n_agents_left += -1


    def walk_tangent(self):
        if self.transition1 == True:
            if [self.xcor // 1, self.ycor // 1] != self.h:
                self.fd(self.wspeed)
            else:
                self.transition1 = False
                self.follow_wall = True
                while self.wall(self.heading, self.wspeed) != 'nobody':
                    self.turn_positive(self.direction * 44.5 / 180 * np.pi)
                if round(self.heading / (np.pi / 2)) == 0 or round(self.heading / (np.pi / 2)) == 4:
                    self.heading = 0
                elif round(self.heading / (np.pi / 2)) == 1:
                    self.heading = np.pi / 2
                elif round(self.heading / (np.pi / 2)) == 2:
                    self.heading = np.pi
                else:
                    self.heading = 3 * np.pi / 2

        else:
            if self.transition2 == True:
                if distance2v([self.xcor, self.ycor], [self.destination]) <= self.d_min:
                    self.face_target(self.destination)
                    self.transition2 = False
                    self.h = 'nobody'
                else:
                    self.fd(self.wspeed)
            else:
                if self.follow_wall == True:
                    self.follow_boundry()
                else:
                    self.motion_toward_target()


    def follow_boundry(self):
        self.LTG()
        c1 = self.wall_p(self.heading - np.pi / 2 * self.direction, 1)
        c2 = self.wall_p(self.heading - 135 / 180 * np.pi * self.direction, 1)
        if c1 == 'nobody' and c2 != 'nobody':
            self.turn_positive(- self.direction * np.pi / 2)
        next_step = self.wall(self.heading, self.wspeed)
        if next_step != 'nobody':
            if self.wall(self.heading, 1) == [self.xcor // 1, self.ycor // 1] and self.wall(
                            self.heading + self.direction * np.pi / 2, 1) != 'nobody':
                self.direction = -self.direction
                self.turn_positive(np.pi)
            else:
                while self.wall(self.heading, self.wspeed) != 'nobody':
                    self.turn_positive(self.direction * np.pi / 2)
                tmp = self.heading
                self.face_target(self.destination)
                if self.target_visible(self.destination) or self.find_wall(self.heading, self.r) == 'nobody':
                    self.follow_wall = False
                    self.h = 'nobody'
                else:
                    self.heading = tmp
        else:
            if distance2v([self.xcor, self.ycor], self.destination) <= 1:
                self.xcor = self.destination[0]
                self.ycor = self.destination[1]
                n_agents_left += -1
                active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                del self
            ### add human interaction
            self.fd(self.wspeed)
            if distance2v([self.xcor, self.ycor], self.destination) < self.d_min:
                self.d_min = distance2v([self.xcor, self.ycor], self.destination)
            x2 = (self.xcor + self.wspeed * np.cos(self.heading + np.pi)) // 1
            y2 = (self.ycor + self.wspeed * np.sin(self.heading + np.pi)) // 1
            if [self.xcor // 1, self.ycor // 1] == self.h and [x2, y2] != self.h:
                active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                del self
                n_trapped += 1
                n_agents_left -= 1
            elif [self.xcor // 1, self.ycor // 1] == self.destination:
                active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
                del self
                n_agents_left += -1
            else:
                dist = size * 2
                node = []
                for i in range(len(self.ltg_list)):
                    dist2 = distance2v(self.ltg_list[i], self.destination)
                    if dist2 < dist:
                        node = self.ltg_list[i]
                        dist = dist2
                print(node[0]-35,node[1]-35)
                if distance2v(node, self.destination) < self.d_min:
                    self.follow_wall = False
                    self.transition2 = True
                    self.face_target(node)

                tmp = self.heading
                self.face_target(self.destination)
                if self.target_visible(self.destination) or self.find_wall(self.heading, self.r) == 'nobody':
                    self.follow_wall = False
                    self.h = 'nobody'
                else:
                    self.heading = tmp

    def motion_toward_target(self):
        if [self.xcor // 1, self.ycor // 1] == self.destination:
            n_agents_left += -1
            active_agents = np.delete(active_agents, np.nonzero(active_agents == self.ID)[0][0])
            del self
        else:
            tmp = self.heading
            self.face_target(self.destination)
            if self.target_visible(self.destination) or self.find_wall(self.heading, self.r) == 'nobody':
                ### add human interaction
                self.fd(self.wspeed)
            else:
                self.heading = tmp
                self.LTG()
                dist = size * 2
                node = []
                for i in range(len(self.ltg_list)):
                    dist2 = distance2v(self.ltg_list[i], self.destination) + distance2v(self.ltg_list[i],[self.xcor//1, self.ycor//1])
                    if dist2 < dist:
                        node = self.ltg_list[i]
                        dist = dist2
                self.face_target(node)
                print(node[0] - 35, node[1] - 35)
                if distance2v(node, self.destination) >= distance2v([self.xcor, self.ycor], self.destination):
                    self.h = node
                    tmp = self.heading
                    self.face_target(self.destination)
                    tmp2 = self.find_wall(self.heading, self.r)
                    self.heading = tmp
                    self.dmin = distance2v(tmp2, self.destination)
                    if node[0] < tmp2[0] or node[1] > tmp2[1]:
                        self.direction = 1
                    else:
                        self.direction = -1
                    self.transition1 = True
                else:
                    next_step = self.wall(self.heading, self.wspeed)
                    if next_step != 'nobody':
                        dist = size * 2
                        tmp = []
                        for s in [-1, 0, 1]:
                            for t in [-1, 0, 1]:
                                dist2 = distance2v([self.xcor // 1 + s, self.ycor // 1 + t], [self.destination])
                                if patch_list[int((self.xcor // 1 + s) + (self.ycor // 1 + t) * size)][2] != (
                                        0.0, 0.0, 0.0, 1.0) and dist2 < dist:
                                    tmp = [self.xcor // 1 + s, self.ycor // 1 + t]
                                    dist = dist2
                        self.xcor = tmp[0]
                        self.ycor = tmp[1]
                    else:
                        self.fd(self.wspeed)

    def LTG(self):
        self.ltg_list = []
        tmp = self.heading
        self.face_target(self.destination)
        direct = self.heading
        self.heading = tmp
        if self.find_wall(direct, self.r) == 'nobody':
            x2 = (self.xcor + self.wspeed * np.cos(self.heading + direct)) // 1
            y2 = (self.ycor + self.wspeed * np.sin(self.heading + direct)) // 1
            self.ltg_list.append([x2, y2])
        vis = []
        for s in range(-(int(self.r)), int(self.r) + 1):
            for t in range(-(int(self.r)), int(self.r) + 1):
                if s != 0 or t != 0:
                    if self.xcor // 1 + s < size and self.xcor // 1 + s >= 0 and self.ycor // 1 + t < size and self.ycor // 1 + t >= 0:
                        if distance2v([self.xcor, self.ycor],
                                      [self.xcor // 1 + s, self.ycor // 1 + t]) < self.r and self.target_visible(
                            [self.xcor // 1 + s, self.ycor // 1 + t]):
                            vis.append([self.xcor // 1 + s, self.ycor // 1 + t])
        edge1 = []
        for i in vis:
            nbrs = []
            for s in [-1, 0, 1]:
                for t in [-1, 0, 1]:
                    if (s != 0 or t != 0) and (
                                        i[0] + s < size and i[0] + s >= 0 and i[1] + t < size and i[1] + t >= 0):
                        nbrs.append([i[0] + s, i[1] + t])
            k = 0
            ch = False
            while k < len(nbrs) and ch == False:
                if patch_list[int((nbrs[k][0]) + (nbrs[k][1]) * size)][2] == (0.0, 0.0, 0.0, 1.0):
                    ch = True
                else:
                    k = k + 1
            if ch == True:
                edge1.append(i)

        for i in edge1:
            count = 0
            for s in [-1, 0, 1]:
                for t in [-1, 0, 1]:
                    if (s == 0 or t == 0) and (s != 0 or t != 0):
                        if [i[0] + s, i[1] + t] in edge1:
                            count += 1
            if count == 1:
                self.ltg_list.append(i)
        self.ltg_list = np.unique(self.ltg_list, axis=0)
        self.ltg_list = self.ltg_list.tolist()


def distance2v(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


# ########### GUI (wxpython) #############
# class MyFrame(wx.Frame):
#     def __init__(self, *args, **kwds):
#         kwds["style"] = kwds.get("style", 0) | wx.DEFAULT_FRAME_STYLE
#         wx.Frame.__init__(self, *args, **kwds)
#         self.SetSize((385, 385))
#         self.choice_2 = wx.Choice(self, wx.ID_ANY, choices=["L", "U1", "U2", "pixelated", "T-corridor", "closed"])
#         self.checkbox_1 = wx.CheckBox(self, wx.ID_ANY, "Extension 1")
#         self.checkbox_2 = wx.CheckBox(self, wx.ID_ANY, "Extension 2")
#         self.choice_1 = wx.Choice(self, wx.ID_ANY, choices=["COPE0", "Bug1", "Bug2", "DistBug", "KBug", "TangentBug"])
#         self.checkbox_3 = wx.CheckBox(self, wx.ID_ANY, "Extension 3")
#         self.text_ctrl_1 = wx.TextCtrl(self, wx.ID_ANY, "1")
#         self.button_1 = wx.Button(self, wx.ID_ANY, "GO!")
#         self.text_ctrl_2 = wx.TextCtrl(self, wx.ID_ANY, "1")
#         self.button_2 = wx.Button(self, wx.ID_CLOSE, "")

#         self.__set_properties()
#         self.__do_layout()

#         self.button_1.Bind(wx.EVT_BUTTON, self.ongo)
#         self.button_2.Bind(wx.EVT_BUTTON, self.onclose)
#         # end wxGlade

#     def onclose(self, event):
#         self.Close()
#         quit()

#     def ongo(self, event):
#         global algorithm, obs, n_agents, runs, extension1, extension2, extension3
#         algorithm = self.choice_1.GetString(self.choice_1.GetSelection())
#         obs = self.choice_2.GetString(self.choice_2.GetSelection())
#         n_agents = int(self.text_ctrl_1.GetValue())
#         runs = int(self.text_ctrl_2.GetValue())
#         extension1 = self.checkbox_1.GetValue()
#         extension2 = self.checkbox_2.GetValue()
#         extension3 = self.checkbox_3.GetValue()
#         self.Close()

#     def __set_properties(self):
#         # begin wxGlade: MyFrame.__set_properties
#         self.SetTitle("frame")
#         self.choice_2.SetMinSize((110, 25))
#         self.choice_2.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         self.choice_2.SetSelection(0)
#         self.checkbox_1.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         self.checkbox_2.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         self.choice_1.SetMinSize((115, 25))
#         self.choice_1.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         self.choice_1.SetSelection(0)
#         self.checkbox_3.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         self.text_ctrl_1.SetMinSize((100, 25))
#         self.text_ctrl_1.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         self.button_1.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         self.text_ctrl_2.SetMinSize((100, 25))
#         self.text_ctrl_2.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         self.button_2.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         # end wxGlade

#     def __do_layout(self):
#         # begin wxGlade: MyFrame.__do_layout
#         sizer_1 = wx.BoxSizer(wx.VERTICAL)
#         grid_sizer_1 = wx.GridSizer(12, 2, 0, 0)
#         label_1 = wx.StaticText(self, wx.ID_ANY, "Choose obstacle")
#         label_1.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         grid_sizer_1.Add(label_1, 0, wx.ALIGN_CENTER, 0)
#         grid_sizer_1.Add((0, 0), 0, 0, 0)
#         grid_sizer_1.Add(self.choice_2, 0, wx.ALIGN_CENTER, 0)
#         label_5 = wx.StaticText(self, wx.ID_ANY, "For DistBug:")
#         label_5.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         grid_sizer_1.Add(label_5, 0, 0, 0)
#         grid_sizer_1.Add((0, 0), 0, 0, 0)
#         grid_sizer_1.Add(self.checkbox_1, 0, 0, 0)
#         label_2 = wx.StaticText(self, wx.ID_ANY, "Choose algorithm")
#         label_2.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         grid_sizer_1.Add(label_2, 0, wx.ALIGN_CENTER, 0)
#         grid_sizer_1.Add(self.checkbox_2, 0, 0, 0)
#         grid_sizer_1.Add(self.choice_1, 0, wx.ALIGN_CENTER, 0)
#         grid_sizer_1.Add(self.checkbox_3, 0, 0, 0)
#         grid_sizer_1.Add((0, 0), 0, 0, 0)
#         grid_sizer_1.Add((0, 0), 0, 0, 0)
#         label_3 = wx.StaticText(self, wx.ID_ANY, "Number of agents")
#         label_3.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         grid_sizer_1.Add(label_3, 0, wx.ALIGN_CENTER, 0)
#         grid_sizer_1.Add((0, 0), 0, 0, 0)
#         grid_sizer_1.Add(self.text_ctrl_1, 0, wx.ALIGN_CENTER, 0)
#         grid_sizer_1.Add(self.button_1, 0, wx.ALIGN_CENTER, 0)
#         grid_sizer_1.Add((0, 0), 0, 0, 0)
#         grid_sizer_1.Add((0, 0), 0, 0, 0)
#         label_4 = wx.StaticText(self, wx.ID_ANY, "Number of runs")
#         label_4.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
#         grid_sizer_1.Add(label_4, 0, wx.ALIGN_CENTER, 0)
#         grid_sizer_1.Add((0, 0), 0, 0, 0)
#         grid_sizer_1.Add(self.text_ctrl_2, 0, wx.ALIGN_CENTER, 0)
#         grid_sizer_1.Add(self.button_2, 0, wx.ALIGN_CENTER, 0)
#         grid_sizer_1.Add((0, 0), 0, 0, 0)
#         grid_sizer_1.Add((0, 0), 0, 0, 0)
#         sizer_1.Add(grid_sizer_1, 1, wx.EXPAND, 0)
#         self.SetSizer(sizer_1)
#         self.Layout()
#         # end wxGlade
# # end of class MyFrame


# class MyApp(wx.App):
#     def OnInit(self):
#         self.frame = MyFrame(None, wx.ID_ANY, "")
#         self.SetTopWindow(self.frame)
#         self.frame.Show()
#         return True


# app = MyApp(0)
# app.MainLoop()
# #######

# ########### GUI (tkinter) #############
root = Tk()
# root.geometry('270x140')
root.title("Input parameters")
# Create a Tkinter variable
tkvar = StringVar(root)
# Dictionary with options
obs_options = ['L', 'U1', 'U2', 'pixelated', 'T-corridor', 'closed']
tkvar.set(obs_options[0])  # set the default option
popupMenu = OptionMenu(root, tkvar, *obs_options)
Label(root, text="Choose obstacle").grid(row=1, column=1)
popupMenu.grid(row=2, column=1)

tkvar2 = StringVar(root)
alg_options = ['COPE0', 'Bug2', 'DistBug', 'Bug1', 'KBug', 'TangentBug']
tkvar2.set(alg_options[0])  # set the default option
popupMenu2 = OptionMenu(root, tkvar2, *alg_options)
Label(root, text="Choose algorithm").grid(row=3, column=1)
popupMenu2.grid(row=4, column=1)

var1 = IntVar()
var2 = IntVar()
var3 = IntVar()
Label(root, text="For DistBug:").grid(row=2, column=2)
c1 = Checkbutton(root, text="Extension 1", variable=var1)
c1.grid(row=3, column=2, columnspan=2, sticky=W, padx=10)
c2 = Checkbutton(root, text="Extension 2", variable=var2)
c2.grid(row=4, column=2, columnspan=2, sticky=W, padx=10)
c3 = Checkbutton(root, text="Extension 3", variable=var3)
c3.grid(row=5, column=2, columnspan=2, sticky=W, padx=10)

x = StringVar()
x.set('1')
e1 = Entry(root, textvariable=x)
Label(root, text="Number of agents").grid(row=5, column=1)
e1.grid(row=6, column=1)
x2 = StringVar()
x2.set('1')
e2 = Entry(root, textvariable=x2)
Label(root, text="Number of runs").grid(row=7, column=1)
e2.grid(row=8, column=1)

Button(root, text='     GO!     ', command=root.quit).grid(row=10, column=0, sticky=W, padx=10, pady=10)
Button(root, text='     Quit     ', command=root.destroy).grid(row=10, column=2, sticky=W, padx = 10, pady=10)

mainloop()
obs = tkvar.get()
n_agents = int(e1.get())
runs = int(e2.get())
algorithm = tkvar2.get()
extension1 = var1.get()
extension2 = var2.get()
extension3 = var3.get()
root.destroy()
# ################## input data ##################################

size = 71
if obs == 'L' or obs == 'pixelated' or obs == 'T-corridor':
    destination = [35, 60]  # top
elif obs == 'U1':
    destination = [35, 10]  # down
elif obs == 'closed':
    destination = [35, 35]  # trapped
else:
    destination = [35, 25]  # middle


# ####### Build geometry ###########
plt.figure()
ax = plt.axes()
plt.xlim((0, size))
plt.ylim((0, size))
ax.set_aspect('equal')
plt.ion()

if obs == 'L':
    coord = [[(10, 51), (11, 51), (11, 26), (61, 26), (61, 25), (10, 25)]]
elif obs == 'U1' or obs == 'U2':
    coord = [[(25, 41), (26, 41), (26, 21), (45, 21), (45, 41), (46, 41), (46, 20), (25, 20)]]
elif obs == 'pixelated':
    coord = [[(10, 50), (60, 50), (10, 20)]]
elif obs == 'T-corridor':
    coord1 = [(20, 35), (35, 35), (35, 20), (20, 20)]
    coord2 = [(36, 35), (51, 35), (51, 20), (36, 20)]
    coord3 = [(28, 51), (43, 51), (43, 36), (28, 36)]
    coord = [coord1, coord2, coord3]
elif obs == 'closed':
    coord = [[(25, 51), (46, 51), (46, 20), (25, 20)]]

obs_list = []
obs_path = []
for c in coord:
    obs_list.append(c)
    obs_path.append(Path(c))
    my_shape = Polygon(c)
    p = PolygonPatch(my_shape, color='black')
    ax.add_patch(p)
if obs == 'closed':
    coord2 = [(26, 50), (45, 50), (45, 21), (26, 21)]
    my_shape = Polygon(coord2)
    p = PolygonPatch(my_shape, color='white')
    ax.add_patch(p)


ax.add_patch(patches.CirclePolygon(destination, radius=1, resolution=5, color='red'))

# ##### build boundary polygons for obstacles
bound_list = []
bound_path = []

for j in range(len(obs_list)):
    nodes2 = []
    nodes = obs_list[j]
    for i in range(len(nodes)-1):
        L1 = np.sqrt((nodes[i][0]-nodes[i-1][0])**2+(nodes[i][1]-nodes[i-1][1])**2)
        L2 = np.sqrt((nodes[i][0]-nodes[i+1][0])**2+(nodes[i][1]-nodes[i+1][1])**2)
        if L1 != 0 and L2 != 0:
            arrow1 = [(nodes[i][0]-nodes[i-1][0])/L1*0.5, (nodes[i][1]-nodes[i-1][1])/L1*0.5]
            arrow2 = [(nodes[i][0]-nodes[i+1][0])/L2*0.5, (nodes[i][1]-nodes[i+1][1])/L2*0.5]
            arrow = [x+y for x, y in zip(arrow1, arrow2)]
            new_node = [x+y for x, y in zip(arrow, nodes[i])]
            if obs_path[j].contains_point(new_node):
                arrow = [x*y for x, y in zip(arrow, [-1, -1])]
                new_node = [x+y for x, y in zip(arrow, nodes[i])]
            nodes2.append(new_node)
    i = len(nodes)-1
    L1 = np.sqrt((nodes[i][0]-nodes[i-1][0])**2+(nodes[i][1]-nodes[i-1][1])**2)
    L2 = np.sqrt((nodes[i][0]-nodes[0][0])**2+(nodes[i][1]-nodes[0][1])**2)
    arrow1 = [(nodes[i][0]-nodes[i-1][0])/L1*0.5, (nodes[i][1]-nodes[i-1][1])/L1*0.5]
    arrow2 = [(nodes[i][0]-nodes[0][0])/L2*0.5, (nodes[i][1]-nodes[0][1])/L2*0.5]
    arrow = [x+y for x, y in zip(arrow1, arrow2)]
    new_node = [x + y for x, y in zip(arrow, nodes[i])]
    if obs_path[j].contains_point(new_node):
        arrow = [x * y for x, y in zip(arrow, [-1, -1])]
        new_node = [x + y for x, y in zip(arrow, nodes[i])]
    nodes2.append(new_node)
    bound_list.append(nodes2)
    bound_path.append(Path(nodes2))
    my_shape = Polygon(nodes2)
    p = PolygonPatch(my_shape, ec='blue', fc='none')
    ax.add_patch(p)

# ####### Make agents ###########
for r in range(runs):
    start = time.time()
    agents_list = {}
    active_agents = np.arange(n_agents)
    n_trapped = 0
    n_agents_left = n_agents
    for k in range(n_agents):
        if obs == 'L' or obs == 'pixelated' or obs == 'T-corridor' or obs == 'U2':
            agents_list[k] = Agent(k, random.random()*70, random.random()*19, destination)
        elif obs == 'U1':
            agents_list[k] = Agent(k, random.random()*70, random.random()*30+40, destination)
        else:
            agents_list[k] = Agent(k, random.random()*19+random.randint(0, 1)*50, random.random()*19+random.randint(0, 1)*50, destination)
        pos = ax.scatter(agents_list[k].xcor, agents_list[k].ycor, color='black', s=20)

# ####### Run ###########
    n_steps = 0
    while len(active_agents) > 0:
        n_steps += 1
        for j in active_agents:
            ax.scatter(agents_list[j].xcor, agents_list[j].ycor, color='grey', s=5, zorder=2)  # marker='.'
            if algorithm == 'COPE0':
                agents_list[j].walk_cope0()
            elif algorithm == 'Bug2':
                agents_list[j].walk_bug2()
            elif algorithm == 'DistBug':
                agents_list[j].walk_distbug()
            elif algorithm == 'Bug1':
                agents_list[j].walk_bug1()
            elif algorithm == 'KBug':
                agents_list[j].walk_kbug()
            elif algorithm == 'TangentBug':
                agents_list[j].walk_tangent()
            # plt.plot([agents_list[j].xcor-np.cos(agents_list[j].heading)*0.5,agents_list[j].xcor+np.cos(agents_list[j].heading)*0.5],[agents_list[j].ycor-np.sin(agents_list[j].heading)*0.5,agents_list[j].ycor+np.sin(agents_list[j].heading)*0.5])
            # plt.plot([agents_list[j].pre_node[0],agents_list[j].xcor],[agents_list[j].pre_node[1],agents_list[j].ycor],color='grey')
            ax.scatter(agents_list[j].xcor, agents_list[j].ycor, color='green', s=5, zorder=2)  # marker='.'
        plt.pause(0.05)
        if n_steps > 500:
            active_agents = {}
            print("Not converging!")

    end = time.time()
    print('Timer = ', end - start)
    print('steps = ', n_steps)
    print('# trapped agents = ', n_trapped)




