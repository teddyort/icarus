#!/usr/bin/env python
import rospy
import numpy as np
from robot_driver.srv import MoveCartesian
from geometry_msgs.msg import PointStamped

# Screensaver Node
# Authors: Teddy Ort
# Inputs: 
# Outputs: 

class ScreensaverNode(object):
    def __init__(self):
        self.node_name = 'screensaver_node'
        # Setup Params
        xlim   = self.setupParameter("/robot_driver_node/workspace/x_lim", [0.05, 0.05])
        ylim   = self.setupParameter("/robot_driver_node/workspace/y_lim",[0,0])
        zlim   = self.setupParameter("/robot_driver_node/workspace/z_lim",[0,0])
        rospy.loginfo("[%s] has started", self.node_name)

        # Wait for service server
        rospy.loginfo("[%s] Waiting for robot_driver move_cartesian service", self.node_name)
        move_cart_name = "/robot_driver_node/move_cartesian"
        rospy.wait_for_service(move_cart_name)

        move_cart = rospy.ServiceProxy(move_cart_name, MoveCartesian)
        ps = PointStamped()
        ps.header.frame_id = "world"
        while not rospy.is_shutdown():
            # Choose a random position in the workspace
            ps.point.x = np.random.uniform(xlim[0], xlim[1])
            ps.point.y = np.random.uniform(ylim[0], ylim[1])
            ps.point.z = np.random.uniform(zlim[0], zlim[1])
            rospy.loginfo("Going to: %s", ps.point)
            move_cart(ps)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('screensaver_node', anonymous=False)
    forward_kinematics_node = ScreensaverNode()
    rospy.spin()