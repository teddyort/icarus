#!/usr/bin/env python
import rospy
import unittest, rostest
from std_msgs.msg import Int8, Float32
import numpy as np
from std_srvs.srv import Trigger

class RulaFilterNode(unittest.TestCase):
    def __init__(self, *args):
        super(RulaFilterNode, self).__init__(*args)

    def setup(self):
        # Setup the node
        rospy.init_node('rula_filter_tester_node', anonymous=False)
        self.msg_filtered_score = Float32()
        self.msg_count = 0

        # Setup the publisher and subscriber
        self.pub_raw_score = rospy.Publisher("~raw_score", Int8, queue_size=10, latch=True)
        self.sub_filtered_score = rospy.Subscriber("~filtered_score", Float32, self.filtered_scoreCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5)  # Wait at most 5 seconds for the node to come up
        while (self.pub_raw_score.get_num_connections() < 1 or self.sub_filtered_score.get_num_connections() < 1) and \
                not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLessEqual(rospy.Time.now(), timeout, "Connection to node timed out.")

    def filtered_scoreCallback(self, msg_filtered_score):
        self.msg_filtered_score = msg_filtered_score
        self.msg_count += 1

    def test_publisher_and_subscriber(self):
        self.setup()  # Setup the node
        self.assertGreaterEqual(self.pub_raw_score.get_num_connections(), 1, "No connections found on raw_score topic")
        self.assertGreaterEqual(self.sub_filtered_score.get_num_connections(), 1, "No connections found on filtered_score topic")

    def test_average_score(self):
        self.setup()

        # Publish some messages
        msg_raw = Int8()
        rate = rospy.Rate(10)
        num_msgs = 6
        for i in range(num_msgs):
            msg_raw.data = i
            self.pub_raw_score.publish(msg_raw)
            rate.sleep()

        # Wait for the messages to be received
        timeout = rospy.Time.now() + rospy.Duration(5)  # Wait at most 5 seconds for the node to come up
        while self.msg_count < num_msgs and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLessEqual(rospy.Time.now(), timeout, "Test timed out while waiting for messages to be received")
        self.assertAlmostEqual(self.msg_filtered_score.data, np.mean(range(num_msgs)))

    def test_reset_timer(self):
        self.setup()

        # Publish some messages
        msg_raw = Int8()
        rate = rospy.Rate(10)
        for i in range(5):
            msg_raw.data = i
            self.pub_raw_score.publish(msg_raw)
            rate.sleep()

        # Wait for the messages to be received
        timeout = rospy.Time.now() + rospy.Duration(5)  # Wait at most 5 seconds for the node to come up
        while self.msg_count < 5 and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLessEqual(rospy.Time.now(), timeout, "Test timed out while waiting for messages to be received")

        # Reset using the service
        reset = rospy.ServiceProxy("rula_filter_node/reset", Trigger)
        reset()

        # Publish some more messages
        for i in range(5):
            msg_raw.data = 2 * i
            self.pub_raw_score.publish(msg_raw)
            rate.sleep()

        # Wait for the messages to be received
        timeout = rospy.Time.now() + rospy.Duration(5)  # Wait at most 5 seconds for the node to come up
        while self.msg_count < 5 and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLessEqual(rospy.Time.now(), timeout, "Test timed out while waiting for messages to be received")
        self.assertAlmostEqual(self.msg_filtered_score.data, 2 * np.mean(range(5)))


if __name__ == '__main__':
    rostest.rosrun('rula', 'rula_filter_tester_node', RulaFilterNode)