#!/usr/bin/env python
import rospy
import unittest, rostest
from robot_driver.srv import *
from geometry_msgs.msg import Vector3Stamped, PointStamped

class RobotDriverTesterNode(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('robot_driver_tester_node', anonymous=False)

        self.move_joint_name = "/robot_driver_node/move_joint"
        self.move_cart_name = "/robot_driver_node/move_cartesian"
        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        try:
            rospy.wait_for_service(self.move_joint_name, 5)
            rospy.wait_for_service(self.move_cart_name, 5)
        except:
            self.assertTrue(False, "Waiting for robot_driver services timed out.")

    def test_robot_driver_connection(self):
        self.setup()

    def test_robot_driver_move_joint(self):
        self.setup()
        vs = Vector3Stamped()
        vs.vector.z = 0.75
        move_joint = rospy.ServiceProxy(self.move_joint_name, MoveJoint)
        move_joint(vs)

    def test_robot_driver_move_cartesian(self):
        self.setup()
        ps = PointStamped()
        ps.point.x = 0.05
        ps.point.y = 0.0
        ps.point.z = 0.0

        move_cart = rospy.ServiceProxy(self.move_cart_name, MoveCartesian)
        resp = move_cart(ps)
        self.assertTrue(resp)

    def test_robot_driver_move_impossible(self):
        self.setup()
        ps = PointStamped()
        ps.point.x = 1.05
        ps.point.y = 0.0
        ps.point.z = 0.0

        move_cart = rospy.ServiceProxy(self.move_cart_name, MoveCartesian)
        resp = move_cart(ps)
        self.assertFalse(resp.success)

if __name__ == '__main__':
    rostest.rosrun('robot_driver', 'robot_driver_tester_node', RobotDriverTesterNode)