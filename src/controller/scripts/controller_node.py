#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool
from dynamixel_msgs.msg import JointState
from robot_driver import RobotDriver

# Default globals
long_term_alpha = 0.1
short_term_alpha = 0.5
grip_close = 3.0
grip_open = 1.8
wrist_rotate = 3.14
range_threshold = 2.0
min_solder_temp = 0#500 === DEBUG TEMP
tin_wait_time = 2
solder_tin_time = 3
dispense_wait_time = 3
solder_dispense_time = 5
load_threshold = 0.006
noload_threshold = 0.001

class ControllerNode(object):
    def __init__(self):
        self.node_name = "controller_node"
        # Setup Params
        self.L0     = self.setupParam("/icarus/L0",0.22) # in meters
        self.L1     = self.setupParam("/icarus/L1",0.22) # in meters
        self.L2     = self.setupParam("/icarus/L2",0.22) # in meters
        
        # Init self.state variables
        self.state = "Init"
        self.temp = 0
        self.holder = True
        self.rng = 100
        self.joint2_load = 0
        self.joint2_avg = 0

        #Create publishers and subscribers
        self.pub_wrist = rospy.Publisher('~wrist_cmd', Float32, queue_size=10)
        self.pub_grip = rospy.Publisher('~grip_cmd', Float32, queue_size=10)
        self.pub_feeder = rospy.Publisher('~feeder_cmd', Bool, queue_size=10)
        self.sub_temp = rospy.Subscriber("~solder/temp", Float32, self.temp_callback)
        self.sub_holder = rospy.Subscriber("~solder/holder", Bool, self.holder_callback)
        self.sub_range = rospy.Subscriber("~range", Float32, self.range_callback)
        self.sub_load = rospy.Subscriber("/joint2_controller/self.state", JointState, self.load_callback)
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

    def controller(self):

        #Create the arm class for controlling dynamixels
        arm = RobotDriver.Driver('arm', self.L0, self.L1, self.L2)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.state == "Init":
                rospy.loginfo("Controller Initialized")
                self.state = "Start"
            elif self.state == "Start":
                rospy.loginfo("self.State: Start")
                arm.move_joint([0.0,0.5, -2.0],2)
                #arm.move_cartesian([0.20, 0.0, 0.11], 2)
                self.pub_grip.publish(grip_open)
                self.pub_wrist.publish(0)
                rospy.sleep(1)
                self.state = "Wait for Range"
                rospy.loginfo("self.State: Wait for Range")
            elif self.state == "Wait for Range":
                if self.rng < range_threshold:
                    rospy.sleep(0.75)
                    self.pub_grip.publish(grip_close)
                    self.state = "Gripped"
                    rospy.loginfo("self.State: Gripped")
            elif self.state == "Gripped":
                if not self.holder:
                    rospy.sleep(1)
                    self.pub_wrist.publish(wrist_rotate)
                    self.state = "Rotated"
                    rospy.loginfo("self.State: Rotated")
            elif self.state == "Rotated":
                if not self.temp > min_solder_temp:
                    rospy.logwarn("Solder too cold. Please turn on to proceed.")
                else:
                    rospy.sleep(tin_wait_time)
                    self.pub_feeder.publish(True)
                    rospy.sleep(solder_tin_time)
                    self.pub_feeder.publish(False)
                    self.state = "Tinned"
                    rospy.loginfo("self.State: Tinned, waiting for load")
            elif self.state == "Tinned":
                #print "Delta", joint2_avg - joint2_load
                if self.joint2_avg - self.joint2_load > load_threshold:
                    rospy.loginfo("Dispensing...")
                    rospy.sleep(dispense_wait_time)
                    self.pub_feeder.publish(True)
                    rospy.sleep(solder_dispense_time)
                    self.pub_feeder.publish(False)
                    self.state = "Dispensed"
                    rospy.loginfo("self.State: Dispensed")
                elif self.holder:
                    self.state = "Retract"
                    rospy.loginfo("self.State: Retract")

            elif self.state == "Dispensed":
                #print "Delta", joint2_avg - joint2_load
                if self.joint2_avg - self.joint2_load < noload_threshold:
                    self.state = "Tinned"
                    rospy.loginfo("self.State: Tinned")
                elif self.holder:
                    self.state = "Retract"
                    rospy.loginfo("self.State: Retract")

            elif self.state == "Retract":
                arm.move_joint([0.0,0.25, -2.25],2)
                self.pub_grip.publish(grip_open)
                rospy.sleep(0.5)
                arm.move_joint([0.0, 0.55, -2.5],2)
                self.state = "Start"
            rate.sleep()

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        # This should call the relax all service of the robot_driver package once that's available
        rospy.loginfo("[%s] shutting down.", self.node_name)

if __name__ == '__main__':
    rospy.init_node('controller_node',anonymous=False)
    controller_node = ControllerNode()
    rospy.on_shutdown(controller_node.onShutdown)
    rospy.spin()