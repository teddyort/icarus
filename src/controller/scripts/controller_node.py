#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool
from dynamixel_msgs.msg import JointState
from robot_driver.srv import MoveCartesian
from geometry_msgs.msg import PointStamped
from controller.Learner import *
from std_srvs.srv import Trigger
import yaml
import rospkg
import csv

# Default globals
start_position = (-0.1, 0, 0.1) # The start position in world frame
long_term_alpha = 0.1
short_term_alpha = 0.5
grip_close = 3.0
grip_open = 0.0
wrist_rotate = 3.14
range_threshold = 2.0
min_solder_temp = 0  # 500 === DEBUG TEMP
tin_wait_time = 2
solder_tin_time = 3
dispense_wait_time = 3
solder_dispense_time = 5
load_threshold = 0.006
noload_threshold = 0.001
min_rula_monitor_time = 3  # 15   # Minimum seconds to count a rula average


class ControllerNode(object):
    def __init__(self):
        self.node_name = "controller_node"

        # Init self.state variables
        self.state = "Init"
        self.temp = 0
        self.holder = True
        self.rng = 100
        self.joint2_load = 0
        self.joint2_avg = 0
        self.rula = 0

        # Setup Params
        self.xlim = self.setupParameter("/robot_driver_node/workspace/x_lim", [0.05, 0.05])
        self.ylim = self.setupParameter("/robot_driver_node/workspace/y_lim", [0, 0])
        self.zlim = self.setupParameter("/robot_driver_node/workspace/z_lim", [0, 0])
        self.learner_continue = self.setupParameter("~continue", "False")

        # If continue is False, reset all the learner params
        if self.learner_continue == False:
            rospy.set_param("~x", start_position)
            rospy.set_param("~fval", 20.0)
            rospy.set_param("~xopt", start_position)
            rospy.set_param("~fopt", 20.0)
            rospy.set_param("~count", 0.0)
            rospy.set_param("~T", 1.0)
        self.learner_x0 = self.setupParameter("~x", start_position)
        self.learner_fval = self.setupParameter("~fval", 20.0)
        self.learner_xopt = self.setupParameter("~xopt", start_position)
        self.learner_fopt = self.setupParameter("~fopt", 20.0)
        self.learner_count = self.setupParameter("~count", 0.0)
        self.learner_T = self.setupParameter("~T", 1.0)

        # Create publishers and subscribers
        self.pub_wrist = rospy.Publisher('~wrist_cmd', Float32, queue_size=10)
        self.pub_grip = rospy.Publisher('~grip_cmd', Float32, queue_size=10)
        self.pub_feeder = rospy.Publisher('~feeder_cmd', Bool, queue_size=10)
        self.sub_temp = rospy.Subscriber("~solder/temp", Float32, self.temp_callback)
        self.sub_holder = rospy.Subscriber("~solder/holder", Bool, self.holder_callback)
        self.sub_range = rospy.Subscriber("~range", Float32, self.range_callback)
        self.sub_load = rospy.Subscriber("~joint2_state", JointState, self.load_callback)
        self.sub_rula = rospy.Subscriber("~filtered_score", Float32, self.rula_callback)

        # Setup the service client for move_cartesian
        # Wait for service server
        rospy.loginfo("[%s] Waiting for robot_driver move_cartesian service", self.node_name)
        move_cart_name = "/robot_driver_node/move_cartesian"
        rospy.wait_for_service(move_cart_name)
        rospy.loginfo("[%s] Move_cartesian service found!", self.node_name)
        self.move_cart = rospy.ServiceProxy(move_cart_name, MoveCartesian)
        self.goal = PointStamped()
        self.goal.header.frame_id = 'world'

        # Setup the service client for rula reset
        rospy.loginfo("[%s] Waiting for rula reset service", self.node_name)
        rula_reset_name = "~rula_reset"
        rospy.wait_for_service(rula_reset_name)
        self.rula_reset = rospy.ServiceProxy(rula_reset_name, Trigger)
        rospy.loginfo("[%s] Rula reset service found!", self.node_name)

        # Setup the learner object from last saved state (or default state)
        lb = [self.xlim[0], self.ylim[0], self.zlim[0]]
        ub = [self.xlim[1], self.ylim[1], self.zlim[1]]
        self.learner = Learner(lb, ub, self.learner_x0)
        self.learner.fval = self.learner_fval
        self.learner.xopt = self.learner_xopt
        self.learner.fopt = self.learner_fopt
        self.learner.count = self.learner_count
        self.learner.T = self.learner_T
        rospy.loginfo("[%s] has started.", self.node_name)
        self.controller()

    def temp_callback(self, msg):
        self.temp = msg.data

    def holder_callback(self, msg):
        self.holder = msg.data

    def range_callback(self, msg):
        self.rng = msg.data

    def load_callback(self, msg):
        self.joint2_load += short_term_alpha * (msg.load - self.joint2_load)
        self.joint2_avg += long_term_alpha * (msg.load - self.joint2_avg)

    def rula_callback(self, msg):
        self.rula = msg.data

    def move(self, goal):
        point = self.goal.point
        (point.x, point.y, point.z) = goal
        return self.move_cart(self.goal).success

    def controller(self):

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.state == "Init":
                rospy.loginfo("[%s] Controller Initialized", self.node_name)
                self.state = "Start"
            elif self.state == "Start":
                rospy.loginfo("[%s] State: %s", self.node_name, self.state)
                self.cur_point = start_position
                self.move(self.cur_point)
                self.pub_grip.publish(grip_open)
                self.pub_wrist.publish(0)
                rospy.sleep(1)
                self.state = "Wait for Range"
                rospy.loginfo("[%s] State: %s", self.node_name, self.state)
            elif self.state == "Wait for Range":
                if self.rng < range_threshold and not self.rng == 0:
                    rospy.sleep(0.75)
                    self.pub_grip.publish(grip_close)
                    self.state = "Gripped"
                    rospy.loginfo("[%s] State: %s", self.node_name, self.state)
            elif self.state == "Gripped":
                if not self.holder:
                    rospy.sleep(1)
                    self.pub_wrist.publish(wrist_rotate)
                    self.state = "Rula_Init"    # Here we switch over to the new net
                    rospy.loginfo("[%s] State: %s", self.node_name, self.state)
            elif self.state == "Rotated":
                if not self.temp > min_solder_temp:
                    rospy.logwarn("Solder too cold. Please turn on to proceed.")
                else:
                    rospy.sleep(tin_wait_time)
                    self.pub_feeder.publish(True)
                    rospy.sleep(solder_tin_time)
                    self.pub_feeder.publish(False)
                    self.state = "Tinned"
                    rospy.loginfo("[%s] State: Tinned, waiting for load", self.node_name)
            elif self.state == "Tinned":
                # print "Delta", joint2_avg - joint2_load
                if self.joint2_avg - self.joint2_load > load_threshold:
                    rospy.loginfo("[%s] Dispensing...", self.node_name)
                    rospy.sleep(dispense_wait_time)
                    self.pub_feeder.publish(True)
                    rospy.sleep(solder_dispense_time)
                    self.pub_feeder.publish(False)
                    self.state = "Dispensed"
                    rospy.loginfo("[%s] State: %s", self.node_name, self.state)
                elif self.holder:
                    self.state = "Retract"
                    rospy.loginfo("[%s] State: Retract", self.node_name)

            elif self.state == "Dispensed":
                # print "Delta", joint2_avg - joint2_load
                if self.joint2_avg - self.joint2_load < noload_threshold:
                    self.state = "Tinned"
                    rospy.loginfo("[%s] State: %s", self.node_name, self.state)
                elif self.holder:
                    self.state = "Retract"
                    rospy.loginfo("[%s] State: %s", self.node_name, self.state)

            elif self.state == "Retract":
                # TODO redo this to use the new services
                pass
                # arm.move_joint([0.0,0.25, -2.25],2)
                # self.pub_grip.publish(grip_open)
                # rospy.sleep(0.5)
                # arm.move_joint([0.0, 0.55, -2.5],2)
                # self.state = "Start"

            # New controller for rula search
            if self.state == "Rula_Init":
                moved = False
                while not moved:
                    self.cur_point = self.learner.get_next_point()
                    rospy.loginfo("[%s] Moving to point: %s", self.node_name, self.cur_point)
                    moved = self.move(self.cur_point)
                rospy.loginfo("[%s] Rula Initialized", self.node_name)
                self.state = "Rula_Waiting"
                rospy.loginfo("[%s] State: %s", self.node_name, self.state)

            elif self.state == "Rula_Waiting":
                if not self.holder:
                    self.rula_start_time = rospy.Time.now()
                    self.rula_reset()
                    self.state = "Rula_Monitoring"
                    rospy.loginfo("[%s] State: %s", self.node_name, self.state)

            elif self.state == "Rula_Monitoring":
                dur = rospy.Time.now() - self.rula_start_time
                if self.holder and dur > rospy.Duration(min_rula_monitor_time):
                    cur_rula=self.rula
                    self.learner.add_sample(self.cur_point, cur_rula)
                    self.dump_params()
                    rospy.loginfo("[%s] Added point: %s \t Rula Score: %s", self.node_name, self.cur_point, cur_rula)
                    moved = False
                    while not moved:
                        self.cur_point = self.learner.get_next_point()
                        rospy.loginfo("[%s] Moving to point: %s", self.node_name, self.cur_point)
                        moved = self.move(self.cur_point)

                    self.state = "Rula_Waiting"
                    rospy.loginfo("[%s] State: %s", self.node_name, self.state)

            rate.sleep()

    def dump_params(self):
        data = {'x': self.learner.x, 'fval': self.learner.fval, 'xopt': self.learner.xopt, 'fopt': self.learner.fopt, 'count': self.learner.count, 'T': self.learner.T}
        rp = rospkg.RosPack()
        with open(rp.get_path("icarus") + "/configs/controller/learner_state.yaml", 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=True))

        # Now append to csv history
        with open(rp.get_path("icarus") + "/output_data.csv", "a") as csvfile:
            wr = csv.writer(csvfile)
            wr.writerow(data['x']+[data['fval']] + data['xopt'] + [data['fopt'], data['count'], data['T']])

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def onShutdown(self):
        # This should call the relax all service of the robot_driver package once that's available
        rospy.loginfo("[%s] shutting down.", self.node_name)


if __name__ == '__main__':
    rospy.init_node('controller_node', anonymous=False)
    controller_node = ControllerNode()
    rospy.on_shutdown(controller_node.onShutdown)
    rospy.spin()
