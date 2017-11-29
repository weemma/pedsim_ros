#! /usr/bin/python
# -*- coding: utf-8 -*-
u"""
Path tracking simulation with pure pursuit steering control and PID speed control.
author: Francisco Marquez Bonilla. Based on the work of Atsushi Sakai
"""
import numpy as np
import math
import matplotlib.pyplot as plt
from unicycle_model import State


class PurePursuit:

    def __init__(self, kp=1.0, lf=1.0, animation=False):

        self.Kp = kp  # speed propotional gain
        self.Lf = lf  # look-ahead distance
        self.animation = animation
        self.state = State(0.0, 0.0, 0.0, 0.0)

    def setState(self, robot_state):

        x = robot_state[0]
        y = robot_state[1]
        v = math.hypot(robot_state[2], robot_state[3])
        yaw = math.atan2(robot_state[3], robot_state[2])
        self.state = State(x, y, yaw, v)

    def setdt(self, dt):
        self.state.setParamdt(dt)

    def setL(self, l):
        self.state.setParaml(l)

    def PIDControl(self, target):
        a = self.Kp * (target - self.state.v)
        return a

    def pure_pursuit_control(self, cx, cy, pind):

        ind = self.calc_target_index(cx, cy)

        if pind >= ind:
            ind = pind

        #  print(pind, ind)
        if ind < len(cx):
            tx = cx[ind]
            ty = cy[ind]
        else:
            tx = cx[-1]
            ty = cy[-1]
            ind = len(cx) - 1

        alpha = math.atan2(ty - self.state.y, tx - self.state.x) - self.state.yaw

        if self.state.v < 0:  # back
            alpha = math.pi - alpha
            #  if alpha > 0:
            #  alpha = math.pi - alpha
            #  else:
            #  alpha = math.pi + alpha

        delta = math.atan2(2.0 * self.state.L * math.sin(alpha) / self.Lf, 1.0)

        return delta, ind

    def calc_target_index(self, cx, cy):
        dx = [self.state.x - icx for icx in cx]
        dy = [self.state.y - icy for icy in cy]

        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]

        ind = d.index(min(d))

        L = 0.0

        while self.Lf > L and (ind + 1) < len(cx):
            dx = cx[ind + 1] - cx[ind]
            dy = cx[ind + 1] - cx[ind]
            L += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1

        return ind

    def closed_loop_prediction(self, cx, cy, speed_profile):

        T = 500.0  # max simulation time
        goal_dis = 0.3
        stop_speed = 0.05

        #  lastIndex = len(cx) - 1
        time = 0.0
        x = [self.state.x]
        y = [self.state.y]
        yaw = [self.state.yaw]
        v = []
        t = [0.0]
        delta = []
        goal = [cx[-1], cy[-1]]
        target_ind = self.calc_target_index( cx, cy)

        while T >= time:
            di, target_ind = self.pure_pursuit_control(cx, cy, target_ind)
            ai = self.PIDControl(speed_profile)
            self.state = self.state.update( ai, di)

            if abs(self.state.v) <= stop_speed:
                target_ind += 1

            time = time + self.state.dt

            # check goal
            dx = self.state.x - goal[0]
            dy = self.state.y - goal[1]
            if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
                # print("Goal")
                break

            x.append(self.state.x)
            y.append(self.state.y)
            yaw.append(self.state.yaw)
            v.append(self.state.v)
            t.append(time)
            delta.append(di)

            if target_ind % 20 == 0 and animation:
                plt.cla()
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(x, y, "ob", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("speed:" + str(round(self.state.v, 2)) +
                          "tind:" + str(target_ind))
                plt.pause(0.0001)

        return v, yaw, t, x, y, delta


    def set_stop_point(self,target_speed, cx, cy, cyaw):
        speed_profile = [target_speed] * len(cx)
        forward = True

        d = []

        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]
            d.append(math.sqrt(dx ** 2.0 + dy ** 2.0))
            iyaw = cyaw[i]
            move_direction = math.atan2(dy, dx)
            is_back = abs(move_direction - iyaw) >= math.pi / 2.0

            if dx == 0.0 and dy == 0.0:
                continue

            if is_back:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

            if is_back and forward:
                speed_profile[i] = 0.0
                forward = False
                #  plt.plot(cx[i], cy[i], "xb")
                #  print(iyaw, move_direction, dx, dy)
            elif not is_back and not forward:
                speed_profile[i] = 0.0
                forward = True
                #  plt.plot(cx[i], cy[i], "xb")
                #  print(iyaw, move_direction, dx, dy)
        speed_profile[0] = 0.0
        speed_profile[-1] = 0.0

        d.append(d[-1])

        return speed_profile, d


    def calc_speed_profile(self, cx, cy, cyaw, target_speed, a):

        speed_profile, d = set_stop_point(target_speed, cx, cy, cyaw)

        nsp = len(speed_profile)

        #  plt.plot(speed_profile, "xb")

        # forward integration
        for i in range(nsp - 1):

            if speed_profile[i + 1] >= 0:  # forward
                tspeed = speed_profile[i] + a * d[i]
                if tspeed <= speed_profile[i + 1]:
                    speed_profile[i + 1] = tspeed
            else:
                tspeed = speed_profile[i] - a * d[i]
                if tspeed >= speed_profile[i + 1]:
                    speed_profile[i + 1] = tspeed

        #  plt.plot(speed_profile, "ok")

        # back integration
        for i in range(nsp - 1):
            if speed_profile[- i - 1] >= 0:  # forward
                tspeed = speed_profile[-i] + a * d[-i]
                if tspeed <= speed_profile[-i - 1]:
                    speed_profile[-i - 1] = tspeed
            else:
                tspeed = speed_profile[-i] - a * d[-i]
                if tspeed >= speed_profile[-i - 1]:
                    speed_profile[-i - 1] = tspeed

        #  flg, ax = plt.subplots(1)
        #  plt.plot(speed_profile, "-r")
        #  plt.show()

        return speed_profile


    # def extend_path(self, cx, cy, cyaw):
    #
    #     dl = 0.1
    #     dl_list = [dl] * (int(self.Lf / self.dl) + 0)
    #
    #     move_direction = math.atan2(cy[-1] - cy[-2], cx[-1] - cx[-2])
    #     is_back = abs(move_direction - cyaw[-1]) >= math.pi / 2.0
    #
    #     for idl in dl_list:
    #         if is_back:
    #             idl *= -1
    #         cx = np.append(cx, cx[-1] + idl * math.cos(cyaw[-1]))
    #         cy = np.append(cy, cy[-1] + idl * math.sin(cyaw[-1]))
    #         cyaw = np.append(cyaw, cyaw[-1])
    #
    #     return cx, cy, cyaw
