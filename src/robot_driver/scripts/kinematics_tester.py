#!/usr/bin/env python

from robot_driver import Kinematics
import math

# kin = Kinematics.Kinematics(0,1,1)
# q = [0, math.pi/2, -math.pi/2]
# p = kin.fkin(q)
# print "The position at " + str(q) + " is " + str(p)
#
# q = kin.ikin(p)
# print "The joint angles to get %s are %s" % (p, q)

kin = Kinematics.Kinematics(0.22,0.22,0.22)
# q = [0, math.pi/2, -math.pi/2]
# p = kin.fkin(q)
# print "The position at " + str(q) + " is " + str(p)
#
p = [0.20, 0.0, 0.11]
q = kin.ikin(p)
print "The joint angles to get %s are %s" % (p, q)