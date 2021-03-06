#!/usr/bin/env python
import rospy
from robot_driver import Kinematics
from robot_driver.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import tf
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
import numpy as np

# Robot Driver Node
# Authors: Teddy Ort
# Inputs: 
# Outputs: 

class RobotDriverNode(object):
    def __init__(self):

        # Retrieve Parameters
        self.node_name = 'robot_driver_node'
        links = self.setupParameter("~links", {'L0': 0.245, 'L1': 0.22, 'L2': 0.24})
        self.workspace = self.setupParameter("~workspace", {'x_lim': 0, 'y_lim': 0, 'z_lim': 0})
        self.max_vel = self.setupParameter("~max_vel", 0.5)

        # Setup the subscriber and publishers
        self.sub_joints = rospy.Subscriber("~joint_states", JointState, self.jointsCallback)
        self.pub_joint1 = rospy.Publisher("~joint1_cmd", Float64, queue_size=1)
        self.pub_joint2 = rospy.Publisher("~joint2_cmd", Float64, queue_size=1)
        self.pub_joint3 = rospy.Publisher("~joint3_cmd", Float64, queue_size=1)

        # Setup the listener
        self.tf = tf.TransformListener()

        # Create the kinematics object
        self.kinematics = Kinematics.Kinematics(links['L0'], links['L1'],links['L2'])

        # Wait until joint states are received to advertise the services
        self.msg_joint_state = None
        while self.msg_joint_state is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Setup the action server
        # self.name = 'arm'
        #
        # self.jta = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # rospy.loginfo("[%s] Waiting for joint trajectory action", self.node_name)
        # self.jta.wait_for_server()
        # rospy.loginfo("[%s] Found joint trajectory action!", self.node_name)

        # Start the service servers
        self.srv = rospy.Service("~move_cartesian", MoveCartesian, self.handle_move_cartesian)
        self.srv = rospy.Service("~move_joint", MoveJoint, self.handle_move_joint)

        rospy.loginfo("[%s] has started", self.node_name)

    def jointsCallback(self, msg_joint_state):
        self.msg_joint_state = msg_joint_state

    def handle_move_joint(self, msg_joints):
        joints = (msg_joints.joints.vector.x, msg_joints.joints.vector.y, msg_joints.joints.vector.z)
        self.move_joint(joints, self.msg_joint_state.position, self.max_vel)
        return True

    def handle_move_cartesian(self, msg_cart):
        point = msg_cart.point
        point.header.frame_id = point.header.frame_id if not point.header.frame_id == "" else "/base_link"

        dest = self.tf.transformPoint("/base_link", point)
        dest = (dest.point.x, dest.point.y, dest.point.z)
        return self.move_cartesian_in_base_frame(dest, self.msg_joint_state.position, self.max_vel)

    def move_joint(self, dest, state, max_v):
        rospy.loginfo("[%s] Moving to angles: %s", self.node_name, dest)

        # Calculate duration
        t = max(abs(np.array(dest) - np.array(state)))/max_v

        # Direct publish method publishes joint commands on all joints (not blocking)
        self.pub_joint1.publish(dest[0])
        self.pub_joint2.publish(dest[1])
        self.pub_joint3.publish(dest[2])

        # # Create the trajectory
        # goal = FollowJointTrajectoryGoal()
        # goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3']
        # point = JointTrajectoryPoint()
        # point.positions = dest
        # point.time_from_start = rospy.Duration(t)
        # goal.trajectory.points.append(point)
        # self.jta.send_goal_and_wait(goal)



    def move_cartesian_in_base_frame(self, pose, state, max_v):
        rospy.loginfo("[%s] Moving to point in base_link frame: %s", self.node_name, pose)
        try:
            angles = self.kinematics.ikin(pose)
            self.move_joint(angles, state, max_v)
            return MoveCartesianResponse(True)
        except ValueError as e:
            rospy.logwarn("[%s] Inverse kineamtics failed to find a solution for the point: %s",self.node_name, pose)
            rospy.logwarn("[%s] Exception Message: %s", self.node_name, e.message)
            raise
            #return MoveCartesianResponse(False)



    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('robot_driver_node', anonymous=False)
    robot_driver_node = RobotDriverNode()
    rospy.spin()