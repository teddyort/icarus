#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import FollowJointTrajectoryFeedback
import math

L1 = 0.22   #meters
L2 = 0.2    #meters
m1 = 0.002  #kilograms
m2 = 0.004  #kilograms
g = 9.81    #gravity

pub2 = None
pub3 = None

def callback(state):
    p = state.actual.positions
    T3 = L2*math.cos(p[2] + p[1])*m2*g
    T2 = T3+L1*math.cos(p[1])*(m1+m2)*g
    rospy.loginfo(rospy.get_caller_id() + "I would command effort %s", [0,T2,T3])
    pub2.publish(T2)
    pub3.publish(T3)
    
def listener():
    global pub2, pub3
    rospy.init_node('antigrav', anonymous=False)
    pub2 = rospy.Publisher("/joint2_controller/command", Float64, queue_size=10)
    pub3 = rospy.Publisher("/joint3_controller/command", Float64, queue_size=10)
    rospy.Subscriber("/arm_controller/state", FollowJointTrajectoryFeedback, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
