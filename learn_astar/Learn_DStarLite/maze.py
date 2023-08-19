# -- coding: utf-8 --

import numpy as np
import heapq

from numpy.random import random_integers as randint
import matplotlib.pyplot as plt


# ����һ��int�͵Ķ�ά���飬1��ʾ�ϰ��0��ʾ���ϰ�
def maze(width, height, complexity=.09, density=.01):
    # Only odd shapes
    shape = (62, 62)#shape=(51,51)Ԫ��
    # Adjust complexity and density relative to maze size
    # complexity = int(complexity * (5 * (shape[0] + shape[1])))#complexity = 45
    # density = int(density * (shape[0] // 2 * shape[1] // 2))#density=6
    # Build actual maze
    z = np.zeros((63,63), dtype=int)#z=51��51�е�0.����
    # Fill borders
    
    # �����ϰ����ѭ��
    for i in range(1000):#�ܹ��õ�density����������
        x, y = randint(0, shape[1] // 2) * 2, randint(0, shape[0] // 2) * 2#0-25����һ������������ٳ���2����x,y,�õ�һ�����������
        z[y, x] = 1 #������ĵ�ͼ������һ
        for j in range(100):#������������Χ����complexity���ϰ���
            neighbours = []
            #���4����Χ���ϰ�������
            if x > 1:           neighbours.append((y, x - 2))
            if x < shape[1] - 2:  neighbours.append((y, x + 2))
            if y > 1:           neighbours.append((y - 2, x))
            if y < shape[0] - 2:  neighbours.append((y + 2, x))

            if len(neighbours):
                #�ڵõ����ϰ������������ѡһ������ֵ1
                y_, x_ = neighbours[randint(0, len(neighbours) - 1)]
                if z[y_, x_] == 0:
                    z[y_, x_] = 1
                    #�ڸ�ֵ1�����ԭ�����ϰ�������֮������긳ֵ1
                    z[y_ + (y - y_) // 2, x_ + (x - x_) // 2] = 1
                    #������������
                    x, y = x_, y_

    z[0, :] = z[62, :] = 1#���͵���һ
    z[:, 0] = z[:, 62] = 1#��ߺ��ұ���һ
    return z


if __name__ == "__main__":
    #�����������
    sx = 1
    sy = 1
    #�����յ�����
    gx = 61
    gy = 61
    #���÷����С
    #grid_size = 100.0

    #�����ϰ����λ��
    ox, oy = [], []#������
    #���ɵ�ͼ
    global_map = maze(width=127, height=127)
    #global_map[global_map == 1] = np.inf
    #����ͼ����Ϊ�ı�
    np.savetxt("./maze.txt", global_map, fmt="%d", delimiter=" ")
    #���ı�����Ϊ��ͼ
    # global_map = np.loadtxt('map/global_map.txt')

    for i in range(0, len(global_map)):#��
        for j in range(0, len(global_map[i])):#��
            if global_map[i][j] == 1:
                #�ϰ�������긳ֵ������
                ox.append(i)
                oy.append(j)
    #��ʾ������
    plt.grid()
    #�����ϰ���
    plt.plot(ox, oy, "sk")
    #������ʼ��
    plt.plot(sx, sy, "og")
    #����Ŀ���
    plt.plot(gx, gy, "xb")
    #����Main����
    #Main(global_map, gx, gy, sx, sy)

    # plt.plot(rx, ry, "-r")
    #��ʾ
    plt.show()