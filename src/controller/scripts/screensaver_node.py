#!/usr/bin/env python
import rospy
import numpy as np
from robot_driver import RobotDriver

# Screensaver Node
# Authors: Teddy Ort
# Inputs: 
# Outputs: 

class ScreensaverNode(object):
    def __init__(self):
        self.node_name = 'screensaver_node'
        # Setup Params
        self.L0     = self.setupParameter("/icarus/L0",0.22) # in meters
        self.L1     = self.setupParameter("/icarus/L1",0.22) # in meters
        self.L2     = self.setupParameter("/icarus/L2",0.22) # in meters
        xlim   = self.setupParameter("/icarus/workspace/x_lim", [0.05, 0.05])
        ylim   = self.setupParameter("/icarus/workspace/y_lim",[0,0])
        zlim   = self.setupParameter("/icarus/workspace/z_lim",[0,0])
        rospy.loginfo("[%s] has started", self.node_name)

        arm = RobotDriver.Driver('arm', self.L0, self.L1, self.L2)
        while not rospy.is_shutdown():
            # Choose a random position in the workspace
            x = np.random.uniform(xlim[0], xlim[1])
            y = np.random.uniform(ylim[0], ylim[1])
            z = np.random.uniform(zlim[0], zlim[1])
            arm.move_cartesian([x, y, z], 4)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('screensaver_node', anonymous=False)
    forward_kinematics_node = ScreensaverNode()
    rospy.spin()