#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, Float32

# Rula Filter Node
# Authors: Teddy Ort
# Inputs: ~raw_score/Int8 - The raw rula score
# Outputs: ~filtered_score/Float32 - The filtered rula score

class RulaFilterNode(object):
    def __init__(self):
        self.node_name = 'rula_filter_node'

        # Setup the publishers and subscribers
        self.sub_raw = rospy.Subscriber("~raw_score", Int8, self.rawCallback)
        self.pub_filtered = rospy.Publisher("~filtered_score", Float32, queue_size=1)

        # Initialize the average
        self.sum = 0
        self.n = 0

        rospy.loginfo("[%s] has started", self.node_name)

    def rawCallback(self, msg_raw):
        self.sum += msg_raw.data
        self.n += 1
        msg_filtered_score = Float32()
        msg_filtered_score.data = self.sum/self.n
        self.pub_filtered.publish(msg_filtered_score)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('rula_filter_node', anonymous=False)
    rula_filter_node = RulaFilterNode()
    rospy.spin()