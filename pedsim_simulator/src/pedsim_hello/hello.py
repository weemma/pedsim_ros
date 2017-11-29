#!/usr/bin/env python

'''
ROS node to connect pedsim with social-lstm

Author : Francisco Marquez Bonilla
Date : 1st September 2017
'''
import sys
sys.path.insert(0, '/home/fran/social-lstm-tf/social_lstm')
import rospy
from pedsim_msgs.msg import TrackedPersons
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from collections import deque
import numpy as np
from social_own_sample import MySampler
from threading import Lock
from pure_pursuit import PurePursuit
import math

mutex_robot = Lock()
mutex_people = Lock()

Hz = 10
desired_speed = 1.0

def say(name):
    print('Hello ' + name)


def talker(topic):

    class PubAndSub:
        '''
        TODO: include mutex
        '''

        def __init__(self):
            rospy.loginfo("INITIALIZING PYTHON MODULE...")
            rospy.init_node('python_talker', anonymous=True)
            self.pub = rospy.Publisher('pedbot/control/cmd_vel', Twist, queue_size=10)
            # Length of the observed traj
            self.obs_len = 5
            # Length of the predicted traj
            self.pred_len = 5
            # Double Queue with positions during obs_seq
            self.people = deque(self.obs_len*[],self.obs_len)
            # Double Queue with positions during obs_seq
            self.ids = deque(self.obs_len*[],self.obs_len)
            # Positions of the robot
            self.robot = deque(self.obs_len*[],self.obs_len)
            # Last state of the robot (x, y, v, theta)
            self.last_state = [0.0, 0.0, 0.0, 0.0]
            # Communication Rate
            self.r = rospy.Rate(Hz)
            # Flag to indicate whether new observations are available
            self.new_data = False
            # Max number of pedestrians
            self.maxNumPeds = 40
            self.model = MySampler()
            rospy.Subscriber("/pedsim/tracked_persons", TrackedPersons, self.callback_persons)
            rospy.Subscriber("/pedsim/robot_position", Odometry, self.callback_robot)

        def callback_persons(self, msg):
            '''
            Callback Function to get position of the near pedestrians
            '''
            mutex_people.acquire()
            self.readcoord(msg)
            self.new_data = True
            mutex_people.release()

        def callback_robot(self, msg):
            '''
            Callback Function to get position of the robot
            TODO: Make sure that pub of robot and tracked robot has same Rate
            '''
            mutex_robot.acquire()
            self.robot.append([-1, msg.pose.pose.position.x, msg.pose.pose.position.y])
            self.last_state = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x, msg.twist.twist.linear.y]
            mutex_robot.release()

        def readcoord(self, msg):
            '''
            Function to get position of the near pedestrians
            TODO: Distance constraint
            TODO: Limit Max # of pedestrian (Send a Warning)
            '''
            positions = []
            ids = []
            for person in msg.tracks:
                positions.append([person.track_id, person.pose.pose.position.x, person.pose.pose.position.y])
                ids.append(person.track_id)
            self.ids.append(ids)
            self.people.append(positions)

        def dummy_model(self, nextPos):
            currPos = self.robot[-1]
            vx = nextPos[1] - currPos[1]
            vy = nextPos[2] - currPos[2]
            theta = math.atan2(vy,vx)
            velocity = math.hypot(vy,vx)
            return velocity, theta

        def next_batch(self):
            '''
            Function to get the next batch of points
            '''

            data = np.zeros((self.obs_len, self.maxNumPeds, 3))
            data[:, 0, :] = np.asarray(self.robot)
            counter = 0
            pedID_list = list(set(x for l in self.ids for x in l))
            numUniquePeds = len(pedID_list)

            for seq in self.people:
                seq_data = np.asarray(seq)
                for ped in range(min(numUniquePeds,self.maxNumPeds-1)):
                    pedID = pedID_list[ped]
                    if pedID == 0:
                        continue
                    else:
                        sped = seq_data[seq_data[:, 0] == pedID, :]
                        if sped.size != 0:
                            data[counter, ped+1, :] = sped

                counter += 1
            return data

        def talk(self):
            counter = 0
            v = []; yaw = []; t = []; x = []; y = []; delta = []
            while not rospy.is_shutdown():
                mutex_robot.acquire()
                mutex_people.acquire()

                if (len(self.people) == self.obs_len) and (len(self.robot) == self.obs_len) and (self.new_data is True):
                    data = self.next_batch()
                    robot_state = self.last_state

                    controller = PurePursuit()
                    controller.setdt(1.0/Hz)
                    controller.setState(robot_state)
                    v = []; yaw = []; t = []; x = []; y = []; delta = []
                    v, yaw, t, x, y, delta = controller.closed_loop_prediction(data[:,1,1], data[:,1,2], desired_speed)
                    counter = 0
                    self.new_data = False

                mutex_people.release()
                mutex_robot.release()

                if counter < len(v)-1:
                    # rospy.loginfo(v[counter])
                    msg = Twist(Vector3(v[counter], 0, 0), Vector3(0, 0, delta[counter]))
                    self.pub.publish(msg)
                    counter += 1
                else:
                    msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                    self.pub.publish(msg)
                self.r.sleep()

    rosboject = PubAndSub()
    rosboject.talk()



    # rospy.spin()
