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
from std_msgs.msg import String
from collections import deque
import numpy as np
from social_own_sample import ownsample


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
            self.pub = rospy.Publisher('dummy', String, queue_size=10)
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
            self.r = rospy.Rate(10)
            # Max number of pedestrians
            self.maxNumPeds = 40
            rospy.Subscriber("/pedsim/tracked_persons", TrackedPersons, self.callback_persons)
            rospy.Subscriber("/pedsim/robot_position", Odometry, self.callback_robot)

        def callback_persons(self, msg):
            '''
            Callback Function to get position of the near pedestrians
            '''
            self.readcoord(msg)

        def callback_robot(self, msg):
            '''
            Callback Function to get position of the robot
            TODO: Make sure that pub of robot and tracked robot has same Rate
            '''
            self.robot.append([-1, msg.pose.pose.position.x, msg.pose.pose.position.y])

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
            while not rospy.is_shutdown():
                if len(self.people) == self.obs_len and len(self.robot) == self.obs_len:
                    data = self.next_batch()
                    output = ownsample(data, 5)
                    rospy.loginfo('Computed Trajectory')
                    rospy.loginfo(output)
                    rospy.loginfo('That's all'')
                    # TODO create an own msg type
                    # self.pub.publish(output)
                self.r.sleep()


    rosboject = PubAndSub()
    rosboject.talk()



    # rospy.spin()
