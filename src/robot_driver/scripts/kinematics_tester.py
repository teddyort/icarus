#!/usr/bin/env python

from robot_driver import Kinematics
import math

kin = Kinematics.Kinematics(1,1)
q = [0, math.pi/2, -math.pi/2]
p = kin.fkin(q)
print "The position at " + str(q) + " is " + str(p)

q = kin.ikin(p)
print "The joint angles to get %s are %s" % (p, q)