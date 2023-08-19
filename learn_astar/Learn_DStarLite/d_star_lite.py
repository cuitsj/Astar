# -- coding: utf-8 --

import numpy as np
import heapq

from numpy.random import random_integers as randint
import matplotlib.pyplot as plt


class Element:
    def __init__(self, key, value1, value2):
        self.key = key
        self.value1 = value1
        self.value2 = value2

    def __eq__(self, other):
        return np.sum(np.abs(self.key - other.key)) == 0

    def __ne__(self, other):
        return self.key != other.key
    #�Ƚ�����Ԫ��Ĵ�С�������ȱȽ�ǰ���ֵ����ȵĻ��ڱȽϺ����ֵ
    def __lt__(self, other):
        return (self.value1, self.value2) < (other.value1, other.value2)

    def __le__(self, other):
        return (self.value1, self.value2) <= (other.value1, other.value2)

    def __gt__(self, other):
        return (self.value1, self.value2) > (other.value1, other.value2)

    def __ge__(self, other):
        return (self.value1, self.value2) >= (other.value1, other.value2)


class DStarLitePlanning:
    #���룺��ͼ��������ꡢ�յ�����
    def __init__(self, r_map, sx, sy, gx, gy):
        self.start = np.array([sx, sy])#�������
        self.goal = np.array([gx, gy])#�յ�����
        self.k_m = 0#Km=0
        #���ɺ͵�ͼ��Сһ���Ķ�ά����1
        self.rhs = np.ones((len(r_map), len(r_map[0]))) * np.inf
        #g=rhs���ֲ�һ��
        self.g = self.rhs.copy()
        #��ȡ��ͼ
        self.global_map = r_map
        #���ɺ͵�ͼ��Сһ���Ķ�ά����0
        self.sensed_map = np.zeros((len(r_map), len(r_map[0])))
        #�յ��rhs��Ϊ0
        self.rhs[self.goal[0], self.goal[1]] = 0
        #���������,��Ϊ��
        self.queue = []
        #����һ���࣬�յ����꣬���յ����������keyֵ��Element
        node = Element(self.goal, *self.CalculateKey(self.goal))
        #������������ʵ�����
        heapq.heappush(self.queue, node)

    #����keyֵ
    def CalculateKey(self, node):
        #��ʼ��keyֵΪ0
        key = [0, 0]
        #key[0]=�ڵ��gֵ��rhs�е���Сֵ+�ڵ������ֱ�߾���+Km
        key[0] = min(self.g[node[0], node[1]], self.rhs[node[0], node[1]]) + self.h_estimate(self.start,node) + self.k_m
        #key[1]=�ڵ��gֵ��rhs�е���Сֵ
        key[1] = min(self.g[node[0], node[1]], self.rhs[node[0], node[1]])
        return key

    def ComputeShortestPath(self):
        #���ѵĳ��ȴ���0 #���˴��õ�<�Ƚ���������أ��Ҷ�������С��keyֵС������keyֵ
        while len(self.queue) > 0 and \
                heapq.nsmallest(1, self.queue)[0] < Element(self.start, *self.CalculateKey(self.start)) or \
                self.rhs[self.start[0], self.start[1]] != self.g[self.start[0], self.start[1]]:#��������rhs!=g
            #keyֵ��С������ֵ��u
            u = heapq.heappop(self.queue).key
            print(u)
            #u������ֵ��g�Ƿ������rhs��ֵ,�ֲ���һ��
            if self.g[u[0], u[1]] > self.rhs[u[0], u[1]]:
                #��g=rhs,�ֲ�һ��
                self.g[u[0], u[1]] = self.rhs[u[0], u[1]]
                #��ȡu�������е��ڵ�ͼ�����ڵ������ֵ
                s_list = self.succ(u)
                for s in s_list:
                    #ѭ�����ã�����u�ĵ�ͼ��������������
                    self.UpdateVertex(s)
            #�ֲ�һ�£��ֲ�Ƿһ��
            else:
                #u��gֵ��Ϊ�����
                self.g[u[0], u[1]] = np.inf
                #��ȡu�������е��ڵ�ͼ�����ڵ������ֵ
                s_list = self.succ(u)
                #�������u�������
                s_list.append(u)
                for s in s_list:
                    self.UpdateVertex(s)

     # fetch successors and predessors
    def succ(self, u):
        #����u������Χ���еĵ�����
        s_list = [np.array([u[0] - 1, u[1] - 1]), np.array([u[0] - 1, u[1]]), np.array([u[0] - 1, u[1] + 1]),
                  np.array([u[0], u[1] - 1]), np.array([u[0], u[1] + 1]), np.array([u[0] + 1, u[1] - 1]),
                  np.array([u[0] + 1, u[1]]), np.array([u[0] + 1, u[1] + 1])]
        row = len(self.global_map)
        col = len(self.global_map[0])
        real_list = []
        for s in s_list:
            #���s�������ڵ�ͼ��Χ��
            if 0 <= s[0] < row and 0 <= s[1] < col:
                real_list.append(s)
        return real_list#����u�������е��ڵ�ͼ�ڵ�����ֵ

    def UpdateVertex(self, u):
        #u�ĵ�ǰ����������յ�������벻Ϊ0
        if np.sum(np.abs(u - self.goal)) != 0:
            #��Ѱ������������������
            s_list = self.succ(u)
            min_s = np.inf
            for s in s_list:
                #���u���u�����ڵ����С��min_s
                if self.cost(u, s) + self.g[s[0], s[1]] < min_s:
                    #����u���u�����ڵ������u�����ڵ�gֵ֮��,���Ҹ�����Сֵ
                    min_s = self.cost(u, s) + self.g[s[0], s[1]]
            #�����Сֵ��Ϊu��rhsֵ
            self.rhs[u[0], u[1]] = min_s
        #���keyΪ00���ڶ�������ɾ��
        if Element(u, 0, 0) in self.queue:
            self.queue.remove(Element(u, 0, 0))
            heapq.heapify(self.queue)
        #���u�����g!=rhs����u���������
        if self.g[u[0], u[1]] != self.rhs[u[0], u[1]]:
            heapq.heappush(self.queue, Element(u, *self.CalculateKey(u)))

   

    # heuristic estimation
    #��������h����ŷ����þ���
    def h_estimate(self, s1, s2):
        #�������������ֵ�Ķ�����
        return np.linalg.norm(s1 - s2)

    # calculate cost between nodes
    def cost(self, u1, u2):
        #���u1��u2����sensed_map����һ������������򷵻������
        if self.sensed_map[u1[0], u1[1]] == np.inf or self.sensed_map[u2[0], u2[1]] == np.inf:
            return np.inf
        #���򣬷���u1��u2��ֱ�߾���
        else:
            return self.h_estimate(u1, u2)

    def sense(self, range_s):
        #����һ��������
        real_list = []
        #��ȡ��ͼ�Ŀ��
        row = len(self.global_map)
        #��ȡ��ͼ�ĳ���
        col = len(self.global_map[0])
        #iѭ����-3��4
        for i in range(-range_s, range_s + 1):
            #jѭ����-3��4
            for j in range(-range_s, range_s + 1):
                #�����������루i,j���ĺ��ڵ�ͼ��Χ��
                if 0 <= self.start[0] + i < row and 0 <= self.start[1] + j < col:
                    #��i��j������0
                    if not (i == 0 and j == 0):
                        #�����������һ������
                        real_list.append(np.array([self.start[0] + i, self.start[1] + j]))
        return real_list#����һ���������ĵ�ͼ��������������


def Main(global_map, gx, gy, sx, sy):
    #����һ����
    node = DStarLitePlanning(global_map, sx, sy, gx, gy)
    #������긳ֵ��last
    last = node.start
    #
    last = ScanAndUpdate(node, last)
    node.ComputeShortestPath()
    while np.sum(np.abs(node.start - node.goal)) != 0:
        s_list = node.succ(node.start)
        min_s = np.inf
        for s in s_list:
            plt.plot(s[0],s[1], 'xy')#���ƻ�ɫ��x����ʾ�����Χ�ĵ�
            if node.cost(node.start, s) + node.g[s[0], s[1]] < min_s:
                min_s = node.cost(node.start, s) + node.g[s[0], s[1]]
                temp = s
        #������ʼ��
        node.start = temp.copy()
        #print(node.start[0], node.start[1])
        #����ʼ��
        plt.plot(node.start[0], node.start[1], '.b')#������ɫ��.����ʾ��㡣
        #����last
        last = ScanAndUpdate(node, last)
        #��ͣ0.1��,ʵʱ��ʾ
        plt.pause(0.1)

#����DStarLitePlanning��ʵ��������һ�������
def ScanAndUpdate(node, last):
    #����sense����������3
    s_list = node.sense(3)
    flag = True
    for s in s_list:
        #�����Χ�ĵ㲻����0��˵�����ϰ���
        if node.global_map[s[0], s[1]] != node.sensed_map[s[0], s[1]]:
            flag = False
            #print('See a wall!')
            break#����ѭ��
    if flag == False:
        #�����ϰ������Km��ֵ
        node.k_m += node.h_estimate(last, node.start)
        #������һ���ֵ
        last = node.start.copy()
        for s in s_list:
            #�����Χ�ĵ㲻����0��˵�����ϰ���
            if node.sensed_map[s[0], s[1]] != node.global_map[s[0], s[1]]:
                plt.plot(s[0],s[1], 'xr')#����ɫ�ġ�
                #���µ�ͼ
                node.sensed_map[s[0], s[1]] = node.global_map[s[0], s[1]]
                #����s���rhsֵ
                node.UpdateVertex(s)
        #�����ȼ��б���ѭ��
        for i in range(len(node.queue)):
            #��ȡ��Сkeyֵ������ֵ
            u = heapq.heappop(node.queue).key
            temp = Element(u, *node.CalculateKey(u))
            heapq.heappush(node.queue, temp)
        heapq.heapify(node.queue)
        node.ComputeShortestPath()
    return last


# randomly generate connected maze
def maze(width, height, complexity=.09, density=.01):
    # Only odd shapes
    shape = ((height // 2) * 2 + 1, (width // 2) * 2 + 1)
    # Adjust complexity and density relative to maze size
    complexity = int(complexity * (5 * (shape[0] + shape[1])))
    density = int(density * (shape[0] // 2 * shape[1] // 2))
    # Build actual maze
    z = np.zeros(shape, dtype=float)
    # Fill borders
    z[0, :] = z[-1, :] = 1
    z[:, 0] = z[:, -1] = 1
    # Make isles
    for i in range(500):
        x, y = randint(0, shape[1] // 2) * 2, randint(0, shape[0] // 2) * 2
        z[y, x] = 1
        for j in range(complexity):
            neighbours = []
            if x > 1:           neighbours.append((y, x - 2))
            if x < shape[1] - 2:  neighbours.append((y, x + 2))
            if y > 1:           neighbours.append((y - 2, x))
            if y < shape[0] - 2:  neighbours.append((y + 2, x))
            if len(neighbours):
                y_, x_ = neighbours[randint(0, len(neighbours) - 1)]
                if z[y_, x_] == 0:
                    z[y_, x_] = 1
                    z[y_ + (y - y_) // 2, x_ + (x - x_) // 2] = 1
                    x, y = x_, y_
    return z


if __name__ == "__main__":
    #�����������
    sx = 1
    sy = 1
    #�����յ�����
    gx = 49
    gy = 49
    #���÷����С
    #grid_size = 100.0

    #�����ϰ����λ��
    ox, oy = [], []
    #���ɵ�ͼ
    global_map = maze(width=50, height=50)
    global_map[global_map == 1] = np.inf
    #����ͼ����Ϊ�ı�
    np.savetxt("global_map.txt", global_map)
    #���ı�����Ϊ��ͼ
    # global_map = np.loadtxt('map/global_map.txt')

    for i in range(0, len(global_map)):
        for j in range(0, len(global_map[i])):
            if global_map[i][j] == np.inf:
                ox.append(i)
                oy.append(j)
    #��ʾ������
    plt.grid(True) 
    #�����ϰ���
    plt.plot(ox, oy, "sk")
    #������ʼ��
    plt.plot(sx, sy, "og")
    #����Ŀ���
    plt.plot(gx, gy, "xb")
    #����Main����
    Main(global_map, gx, gy, sx, sy)

    # plt.plot(rx, ry, "-r")
    #��ʾ
    plt.show()
