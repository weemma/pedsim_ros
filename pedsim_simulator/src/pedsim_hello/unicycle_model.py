#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
author: Francisco Marquez Bonilla. Based on the work of Atsushi Sakai
"""

import math


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1, L = 2.9):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.dt = dt # [s]
        self.L = L  # [m]

    def setParamdt(self, dt):
        self.dt = dt

    def setParaml(self, L):
        self.L = L


    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.v / self.L * math.tan(delta) * self.dt
        self.v += a * self.dt

        return self


# if __name__ == '__main__':
#     print("start unicycle simulation")
#     import matplotlib.pyplot as plt
#
#     T = 100
#     a = [1.0] * T
#     delta = [math.radians(1.0)] * T
#     #  print(delta)
#     #  print(a, delta)
#
#     state = State()
#
#     x = []
#     y = []
#     yaw = []
#     v = []
#
#     for (ai, di) in zip(a, delta):
#         state = update(state, ai, di)
#
#         x.append(state.x)
#         y.append(state.y)
#         yaw.append(state.yaw)
#         v.append(state.v)
#
#     flg, ax = plt.subplots(1)
#     plt.plot(x, y)
#     plt.axis("equal")
#     plt.grid(True)
#
#     flg, ax = plt.subplots(1)
#     plt.plot(v)
#     plt.grid(True)
#
#     plt.show()