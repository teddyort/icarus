#!/usr/bin/env python

import rospy
from dynamixel_controllers.srv import TorqueEnable

def relax():
    print ("Waiting for services...")
    rospy.wait_for_service('/joint1_controller/torque_enable')
    rospy.wait_for_service('/joint2_controller/torque_enable')
    rospy.wait_for_service('/joint3_controller/torque_enable')

    print "Services found. Relaxing..."

    #Create clients
    relax1 = rospy.ServiceProxy('/joint1_controller/torque_enable', TorqueEnable)
    relax2 = rospy.ServiceProxy('/joint2_controller/torque_enable', TorqueEnable)
    relax3 = rospy.ServiceProxy('/joint3_controller/torque_enable', TorqueEnable)

    relax1(False)
    relax2(False)
    relax3(False)
    print "Finished"

if __name__ == '__main__':
    relax()
