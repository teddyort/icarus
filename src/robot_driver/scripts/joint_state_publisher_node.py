#!/usr/bin/env python
import rospy
import numpy as np
from dynamixel_msgs.msg import MotorStateList
from sensor_msgs.msg import JointState

# Joint State Publisher Node
# Authors: Teddy Ort
# Inputs: motor_states:dynamixel_msgs/MotorStateList
# Outputs: joint_states:sensor_msgs/JointState

class JointStatePublisher(object):
    def __init__(self):
        self.node_name = 'joint_state_publisher'
        self.num_motors = self.setupParameter("robot_driver_node/num_motors", 3)

        # setup the publisher and subscriber
        self.sub = rospy.Subscriber("~motor_states", MotorStateList, self.motorStateCallback)
        self.pub = rospy.Publisher("~joint_states", JointState, queue_size=1)

        rospy.loginfo("[%s] has started", self.node_name)

    def motorStateCallback(self, msg_motor_states):
        msg_joint_state = JointState()
        msg_joint_state.header.stamp = rospy.Time.from_sec(msg_motor_states.motor_states[0].timestamp)
        motors_found = 0
        for state in msg_motor_states.motor_states:
            msg_joint_state.name.append("joint" + str(state.id))
            msg_joint_state.position.append((state.position/2048.0-1.0)*np.pi)
            motors_found += 1

        if motors_found == self.num_motors:
            self.pub.publish(msg_joint_state)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('joint_state_publisher', anonymous=False)
    forward_kinematics_node = JointStatePublisher()
    rospy.spin()