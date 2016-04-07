#!/usr/bin/env python
import rospy
import unittest, rostest
from std_msgs.msg import Int8, Float32

class RulaFilterNode(unittest.TestCase):
    def __init__(self, *args):
        super(RulaFilterNode, self).__init__(*args)

    def setup(self):
        # Setup the node
        rospy.init_node('rula_filter_tester_node', anonymous=False)
        self.msg_filtered_score = Float32()
        self.msg_received = False

        # Setup the publisher and subscriber
        self.pub_raw_score = rospy.Publisher("~raw_score", Int8, queue_size=1, latch=True)
        self.sub_filtered_score = rospy.Subscriber("~filtered_score", Float32, self.filtered_scoreCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5)  # Wait at most 5 seconds for the node to come up
        while (self.pub_raw_score.get_num_connections() < 1 or self.sub_filtered_score.get_num_connections() < 1) and \
                not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def filtered_scoreCallback(self, msg_filtered_score):
        self.msg_filtered_score = msg_filtered_score
        self.msg_received = True

    def test_publisher_and_subscriber(self):
        self.setup()  # Setup the node
        self.assertGreaterEqual(self.pub_raw_score.get_num_connections(), 1, "No connections found on raw_score topic")
        self.assertGreaterEqual(self.sub_filtered_score.get_num_connections(), 1, "No connections found on filtered_score topic")

if __name__ == '__main__':
    rostest.rosrun('rula', 'rula_filter_tester_node', RulaFilterNode)