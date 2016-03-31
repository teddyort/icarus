#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool
from dynamixel_msgs.msg import JointState
from robot_driver import RobotDriver

# Default globals
state = "Init"
temp = 0
holder = True
rng = 100
joint2_load = 0
joint2_avg = 0
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

def temp_callback(msg):
    global temp
    temp = msg.data

def holder_callback(msg):
    global holder
    holder = msg.data

def range_callback(msg):
    global rng
    rng = msg.data

def load_callback(msg):
    global joint2_load, joint2_avg
    joint2_load += short_term_alpha * (msg.load - joint2_load)
    joint2_avg += long_term_alpha * (msg.load - joint2_avg)

def controller():
    global state
    rospy.init_node('controller', anonymous=False)

    #Create publishers and subscribers
    pub_wrist = rospy.Publisher('/icarus/wrist_cmd', Float32, queue_size=10)
    pub_grip = rospy.Publisher('/icarus/grip_cmd', Float32, queue_size=10)
    pub_feeder = rospy.Publisher('/icarus/feeder_cmd', Bool, queue_size=10)
    sub_temp = rospy.Subscriber("/icarus/solder/temp", Float32, temp_callback)
    sub_holder = rospy.Subscriber("/icarus/solder/holder", Bool, holder_callback)
    sub_range = rospy.Subscriber("/icarus/range", Float32, range_callback)
    sub_load = rospy.Subscriber("/joint2_controller/state", JointState, load_callback)

    #Create the arm class for controlling dynamixels
    arm = RobotDriver.Driver('arm')

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if state == "Init":
            rospy.loginfo("Controller Initialized")
            state = "Start"
        elif state == "Start":
            rospy.loginfo("State: Start")
            arm.move_joint([0.0,0.5, -2.0],2)
            pub_grip.publish(grip_open)
            pub_wrist.publish(0)
            rospy.sleep(1)
            state = "Wait for Range"
            rospy.loginfo("State: Wait for Range")
        elif state == "Wait for Range":
            if rng < range_threshold:
                rospy.sleep(0.75)
                pub_grip.publish(grip_close)
                state = "Gripped"
                rospy.loginfo("State: Gripped")
        elif state == "Gripped":
            if not holder:
                rospy.sleep(1)
                pub_wrist.publish(wrist_rotate)
                state = "Rotated"
                rospy.loginfo("State: Rotated")            
        elif state == "Rotated":
            if not temp > min_solder_temp:
                rospy.logwarn("Solder too cold. Please turn on to proceed.")
            else:
                rospy.sleep(tin_wait_time)
                pub_feeder.publish(True)
                rospy.sleep(solder_tin_time)
                pub_feeder.publish(False)
                state = "Tinned"
                rospy.loginfo("State: Tinned, waiting for load")
        elif state == "Tinned":
            #print "Delta", joint2_avg - joint2_load
            if joint2_avg - joint2_load > load_threshold:
                rospy.loginfo("Dispensing...")
                rospy.sleep(dispense_wait_time)
                pub_feeder.publish(True)
                rospy.sleep(solder_dispense_time)
                pub_feeder.publish(False)
                state = "Dispensed"
                rospy.loginfo("State: Dispensed")
            elif holder:
                state = "Retract"
                rospy.loginfo("State: Retract")

        elif state == "Dispensed":
            #print "Delta", joint2_avg - joint2_load
            if joint2_avg - joint2_load < noload_threshold:
                state = "Tinned"
                rospy.loginfo("State: Tinned")
            elif holder:
                state = "Retract"
                rospy.loginfo("State: Retract")

        elif state == "Retract":
            arm.move_joint([0.0,0.25, -2.25],2)
            pub_grip.publish(grip_open)
            rospy.sleep(0.5)
            arm.move_joint([0.0, 0.55, -2.5],2)
            state = "Start"

        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
