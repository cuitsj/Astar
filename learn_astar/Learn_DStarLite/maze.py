# -- coding: utf-8 --

import numpy as np
import heapq

from numpy.random import random_integers as randint
import matplotlib.pyplot as plt


# 生成一个int型的二维数组，1表示障碍物，0表示无障碍
def maze(width, height, complexity=.09, density=.01):
    # Only odd shapes
    shape = (62, 62)#shape=(51,51)元组
    # Adjust complexity and density relative to maze size
    # complexity = int(complexity * (5 * (shape[0] + shape[1])))#complexity = 45
    # density = int(density * (shape[0] // 2 * shape[1] // 2))#density=6
    # Build actual maze
    z = np.zeros((63,63), dtype=int)#z=51行51列的0.数组
    # Fill borders
    
    # 产生障碍点的循环
    for i in range(1000):#总共得到density个随机坐标点
        x, y = randint(0, shape[1] // 2) * 2, randint(0, shape[0] // 2) * 2#0-25产生一个随机的数，再乘以2付给x,y,得到一个随机的坐标
        z[y, x] = 1 #这坐标的地图数组置一
        for j in range(100):#在随机坐标点周围生成complexity个障碍点
            neighbours = []
            #最多4个周围的障碍点坐标
            if x > 1:           neighbours.append((y, x - 2))
            if x < shape[1] - 2:  neighbours.append((y, x + 2))
            if y > 1:           neighbours.append((y - 2, x))
            if y < shape[0] - 2:  neighbours.append((y + 2, x))

            if len(neighbours):
                #在得到的障碍点坐标中随机选一个，赋值1
                y_, x_ = neighbours[randint(0, len(neighbours) - 1)]
                if z[y_, x_] == 0:
                    z[y_, x_] = 1
                    #在赋值1坐标和原来的障碍物坐标之间的坐标赋值1
                    z[y_ + (y - y_) // 2, x_ + (x - x_) // 2] = 1
                    #更新随机坐标点
                    x, y = x_, y_

    z[0, :] = z[62, :] = 1#顶和低置一
    z[:, 0] = z[:, 62] = 1#左边和右边置一
    return z


if __name__ == "__main__":
    #设置起点坐标
    sx = 1
    sy = 1
    #设置终点坐标
    gx = 61
    gy = 61
    #设置方格大小
    #grid_size = 100.0

    #设置障碍物的位置
    ox, oy = [], []#空数组
    #生成地图
    global_map = maze(width=127, height=127)
    #global_map[global_map == 1] = np.inf
    #将地图保存为文本
    np.savetxt("./maze.txt", global_map, fmt="%d", delimiter=" ")
    #将文本加载为地图
    # global_map = np.loadtxt('map/global_map.txt')

    for i in range(0, len(global_map)):#行
        for j in range(0, len(global_map[i])):#列
            if global_map[i][j] == 1:
                #障碍点的坐标赋值给数组
                ox.append(i)
                oy.append(j)
    #显示网格线
    plt.grid()
    #绘制障碍物
    plt.plot(ox, oy, "sk")
    #绘制起始点
    plt.plot(sx, sy, "og")
    #绘制目标点
    plt.plot(gx, gy, "xb")
    #调用Main函数
    #Main(global_map, gx, gy, sx, sy)

    # plt.plot(rx, ry, "-r")
    #显示
    plt.show()