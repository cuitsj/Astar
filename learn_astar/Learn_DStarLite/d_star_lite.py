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
    #比较两个元组的大小，会优先比较前面的值，相等的话在比较后面的值
    def __lt__(self, other):
        return (self.value1, self.value2) < (other.value1, other.value2)

    def __le__(self, other):
        return (self.value1, self.value2) <= (other.value1, other.value2)

    def __gt__(self, other):
        return (self.value1, self.value2) > (other.value1, other.value2)

    def __ge__(self, other):
        return (self.value1, self.value2) >= (other.value1, other.value2)


class DStarLitePlanning:
    #输入：地图、起点坐标、终点坐标
    def __init__(self, r_map, sx, sy, gx, gy):
        self.start = np.array([sx, sy])#起点坐标
        self.goal = np.array([gx, gy])#终点坐标
        self.k_m = 0#Km=0
        #生成和地图大小一样的二维矩阵1
        self.rhs = np.ones((len(r_map), len(r_map[0]))) * np.inf
        #g=rhs，局部一致
        self.g = self.rhs.copy()
        #获取地图
        self.global_map = r_map
        #生成和地图大小一样的二维矩阵0
        self.sensed_map = np.zeros((len(r_map), len(r_map[0])))
        #终点的rhs设为0
        self.rhs[self.goal[0], self.goal[1]] = 0
        #定义空数组,作为堆
        self.queue = []
        #例化一个类，终点坐标，和终点坐标的两个key值给Element
        node = Element(self.goal, *self.CalculateKey(self.goal))
        #上面例化的类实例入堆
        heapq.heappush(self.queue, node)

    #计算key值
    def CalculateKey(self, node):
        #初始化key值为0
        key = [0, 0]
        #key[0]=节点的g值和rhs中的最小值+节点和起点的直线距离+Km
        key[0] = min(self.g[node[0], node[1]], self.rhs[node[0], node[1]]) + self.h_estimate(self.start,node) + self.k_m
        #key[1]=节点的g值和rhs中的最小值
        key[1] = min(self.g[node[0], node[1]], self.rhs[node[0], node[1]])
        return key

    def ComputeShortestPath(self):
        #当堆的长度大于0 #（此处用的<比较运算符重载）且堆里面最小的key值小于起点的key值
        while len(self.queue) > 0 and \
                heapq.nsmallest(1, self.queue)[0] < Element(self.start, *self.CalculateKey(self.start)) or \
                self.rhs[self.start[0], self.start[1]] != self.g[self.start[0], self.start[1]]:#或者起点的rhs!=g
            #key值最小的坐标值给u
            u = heapq.heappop(self.queue).key
            print(u)
            #u的坐标值的g是否大于在rhs的值,局部过一致
            if self.g[u[0], u[1]] > self.rhs[u[0], u[1]]:
                #让g=rhs,局部一致
                self.g[u[0], u[1]] = self.rhs[u[0], u[1]]
                #获取u坐标所有的在地图内相邻点的坐标值
                s_list = self.succ(u)
                for s in s_list:
                    #循环调用，传递u的地图内所有相邻坐标
                    self.UpdateVertex(s)
            #局部一致，局部欠一致
            else:
                #u的g值设为无穷大
                self.g[u[0], u[1]] = np.inf
                #获取u坐标所有的在地图内相邻点的坐标值
                s_list = self.succ(u)
                #并添加上u点的坐标
                s_list.append(u)
                for s in s_list:
                    self.UpdateVertex(s)

     # fetch successors and predessors
    def succ(self, u):
        #计算u坐标周围所有的点坐标
        s_list = [np.array([u[0] - 1, u[1] - 1]), np.array([u[0] - 1, u[1]]), np.array([u[0] - 1, u[1] + 1]),
                  np.array([u[0], u[1] - 1]), np.array([u[0], u[1] + 1]), np.array([u[0] + 1, u[1] - 1]),
                  np.array([u[0] + 1, u[1]]), np.array([u[0] + 1, u[1] + 1])]
        row = len(self.global_map)
        col = len(self.global_map[0])
        real_list = []
        for s in s_list:
            #如果s的坐标在地图范围内
            if 0 <= s[0] < row and 0 <= s[1] < col:
                real_list.append(s)
        return real_list#返回u坐标所有的在地图内的坐标值

    def UpdateVertex(self, u):
        #u的当前相邻坐标和终点坐标距离不为0
        if np.sum(np.abs(u - self.goal)) != 0:
            #再寻找这个坐标的相邻坐标
            s_list = self.succ(u)
            min_s = np.inf
            for s in s_list:
                #如果u点和u的相邻点距离小于min_s
                if self.cost(u, s) + self.g[s[0], s[1]] < min_s:
                    #计算u点和u的相邻点距离与u的相邻点g值之和,并且更新最小值
                    min_s = self.cost(u, s) + self.g[s[0], s[1]]
            #这个最小值即为u的rhs值
            self.rhs[u[0], u[1]] = min_s
        #如果key为00的在堆里面则删除
        if Element(u, 0, 0) in self.queue:
            self.queue.remove(Element(u, 0, 0))
            heapq.heapify(self.queue)
        #如果u坐标的g!=rhs，则将u坐标推入堆
        if self.g[u[0], u[1]] != self.rhs[u[0], u[1]]:
            heapq.heappush(self.queue, Element(u, *self.CalculateKey(u)))

   

    # heuristic estimation
    #启发函数h采用欧几里得距离
    def h_estimate(self, s1, s2):
        #计算两个坐标差值的二范数
        return np.linalg.norm(s1 - s2)

    # calculate cost between nodes
    def cost(self, u1, u2):
        #如果u1和u2点在sensed_map内有一个等于无穷大则返回无穷大
        if self.sensed_map[u1[0], u1[1]] == np.inf or self.sensed_map[u2[0], u2[1]] == np.inf:
            return np.inf
        #否则，返回u1和u2的直线距离
        else:
            return self.h_estimate(u1, u2)

    def sense(self, range_s):
        #创建一个空数组
        real_list = []
        #获取地图的宽度
        row = len(self.global_map)
        #获取地图的长度
        col = len(self.global_map[0])
        #i循环从-3到4
        for i in range(-range_s, range_s + 1):
            #j循环从-3到4
            for j in range(-range_s, range_s + 1):
                #如果起点坐标与（i,j）的和在地图范围内
                if 0 <= self.start[0] + i < row and 0 <= self.start[1] + j < col:
                    #且i或j不等于0
                    if not (i == 0 and j == 0):
                        #空数组添加上一个坐标
                        real_list.append(np.array([self.start[0] + i, self.start[1] + j]))
        return real_list#返回一个起点坐标的地图内相邻坐标数组


def Main(global_map, gx, gy, sx, sy):
    #例化一个类
    node = DStarLitePlanning(global_map, sx, sy, gx, gy)
    #起点坐标赋值给last
    last = node.start
    #
    last = ScanAndUpdate(node, last)
    node.ComputeShortestPath()
    while np.sum(np.abs(node.start - node.goal)) != 0:
        s_list = node.succ(node.start)
        min_s = np.inf
        for s in s_list:
            plt.plot(s[0],s[1], 'xy')#绘制黄色的x，表示起点周围的点
            if node.cost(node.start, s) + node.g[s[0], s[1]] < min_s:
                min_s = node.cost(node.start, s) + node.g[s[0], s[1]]
                temp = s
        #更新起始点
        node.start = temp.copy()
        #print(node.start[0], node.start[1])
        #画起始点
        plt.plot(node.start[0], node.start[1], '.b')#绘制蓝色的.，表示起点。
        #调用last
        last = ScanAndUpdate(node, last)
        #暂停0.1秒,实时显示
        plt.pause(0.1)

#传入DStarLitePlanning类实例，和上一起点坐标
def ScanAndUpdate(node, last):
    #调用sense方法，传入3
    s_list = node.sense(3)
    flag = True
    for s in s_list:
        #起点周围的点不等于0，说明有障碍物
        if node.global_map[s[0], s[1]] != node.sensed_map[s[0], s[1]]:
            flag = False
            #print('See a wall!')
            break#结束循环
    if flag == False:
        #遇到障碍物，更新Km的值
        node.k_m += node.h_estimate(last, node.start)
        #更新上一点的值
        last = node.start.copy()
        for s in s_list:
            #起点周围的点不等于0，说明有障碍物
            if node.sensed_map[s[0], s[1]] != node.global_map[s[0], s[1]]:
                plt.plot(s[0],s[1], 'xr')#画红色的×
                #更新地图
                node.sensed_map[s[0], s[1]] = node.global_map[s[0], s[1]]
                #计算s点的rhs值
                node.UpdateVertex(s)
        #在优先级列表里循环
        for i in range(len(node.queue)):
            #获取最小key值得坐标值
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
    #设置起点坐标
    sx = 1
    sy = 1
    #设置终点坐标
    gx = 49
    gy = 49
    #设置方格大小
    #grid_size = 100.0

    #设置障碍物的位置
    ox, oy = [], []
    #生成地图
    global_map = maze(width=50, height=50)
    global_map[global_map == 1] = np.inf
    #将地图保存为文本
    np.savetxt("global_map.txt", global_map)
    #将文本加载为地图
    # global_map = np.loadtxt('map/global_map.txt')

    for i in range(0, len(global_map)):
        for j in range(0, len(global_map[i])):
            if global_map[i][j] == np.inf:
                ox.append(i)
                oy.append(j)
    #显示网格线
    plt.grid(True) 
    #绘制障碍物
    plt.plot(ox, oy, "sk")
    #绘制起始点
    plt.plot(sx, sy, "og")
    #绘制目标点
    plt.plot(gx, gy, "xb")
    #调用Main函数
    Main(global_map, gx, gy, sx, sy)

    # plt.plot(rx, ry, "-r")
    #显示
    plt.show()
