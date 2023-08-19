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

    def __lt__(self, other):
        return (self.value1, self.value2) < (other.value1, other.value2)

    def __le__(self, other):
        return (self.value1, self.value2) <= (other.value1, other.value2)

    def __gt__(self, other):
        return (self.value1, self.value2) > (other.value1, other.value2)

    def __ge__(self, other):
        return (self.value1, self.value2) >= (other.value1, other.value2)



# #����keyֵ
# def CalculateKey(self, node):
#     #��ʼ��keyֵΪ0
#     key = [0, 0]
#     #key[0]=�ڵ��gֵ��rhs�е���Сֵ+�ڵ������ֱ�߾���+Km
#     key[0] = min(self.g[node[0], node[1]], self.rhs[node[0], node[1]]) + self.h_estimate(self.start,node) + self.k_m
#     #key[1]=�ڵ��gֵ��rhs�е���Сֵ
#     key[1] = min(self.g[node[0], node[1]], self.rhs[node[0], node[1]])
#     return key

# # heuristic estimation
# #��������h����ŷ����þ���
# def h_estimate(self, s1, s2):
#     #�������������ֵ�Ķ�����
#     return np.linalg.norm(s1 - s2)

a = Element(0, 2,2)
b = Element(1, 2,10)
print(a>=b)
# a=[49,49]
# print(CalculateKey(np.array([49, 49])))