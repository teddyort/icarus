import rospy, actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from robot_driver import Kinematics

class Driver(object):
        def __init__(self, motor_name, L0, L1, L2):
            self.name = motor_name
            self.kinematics = Kinematics.Kinematics(L0, L1,L2)
            self.jta = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')


        def move_joint(self, angles, t):
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(t)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)

        def move_cartesian(self, pose, t):
            angles = self.kinematics.ikin(pose)
            self.move_joint(angles, t)
            rospy.loginfo("Moving to angles: %s", angles)