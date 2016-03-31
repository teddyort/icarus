#!/usr/bin/env python

__author__ = 'Teddy Ort'

from math import *

class Kinematics(object):
    # The Kinematics class takes as input the length of the two links
    def __init__(self, L1, L2):
        self.L1 = L1
        self.L2 = L2

    # Compute the forward kinematics [x,y,z] for the robot given joint angles [q1, q2, q3] as a list
    def fkin(self, q):
        L1, L2, = self.L1, self.L2
        q1, q2, q3 = q
        rho = L1*cos(q2)+L2*cos(q2+q3)
        x = rho*sin(q1)
        y = L1*sin(q2)+L2*sin(q2+q3)
        z = rho*cos(q1)
        return [x,y,z]

    # Compute the inverse kinematics [q1, q2, q3] for the robot at a given cartesian position [x,y,z]
    def ikin(self, p):
        L1, L2, = self.L1, self.L2
        x,y,z = p

        L = sqrt(x**2+y**2+z**2)
        a = atan2(y, sqrt(x**2+y**2))
        b = acos((L1**2-L2**2+L**2)/(2*L1*L))
        c = pi-acos((L1**2 + L2**2 - L**2) / (2*L1*L2))
        q1 = atan2(x,z)
        q2 = a+b    # a-b for elbow up
        q3 = -c     # c for elbow up
        return [q1, q2, q3]