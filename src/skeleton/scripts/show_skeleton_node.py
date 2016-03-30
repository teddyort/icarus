#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray

def visualize():
    rospy.init_node('show_skeleton', anonymous=False)
    pub = rospy.Publisher("~skeleton", MarkerArray, queue_size=10)
    joint_names = ['shoulder_center', 'head', 'shoulder_left', 'elbow_left', 'wrist_left', 'shoulder_right', 'elbow_right', 'wrist_right']
    joints = MarkerArray()
    for id, name in enumerate(joint_names):
        joints.markers.append(get_new_marker(name, id))

    while not rospy.is_shutdown():
        pub.publish(joints)
        try:
            rospy.sleep(0.1)
        except rospy.ROSTimeMovedBackwardsException: pass

def get_new_marker(name, id):
    m = Marker()
    m.header.frame_id=name
    m.header.stamp = rospy.Time()
    m.id = id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    p = m.pose.position
    o = m.pose.orientation
    c = m.color
    s = m.scale

    p.x, p.y, p.z, o.z, o.y, o.z = (0,)*6
    o.w = 1
    c.a, c.r, c.g, c.b = (1, 0.7, 0.7, 0.7)
    s.x, s.y, s.z = (0.1,)*3
    return m

if __name__ == '__main__':
    visualize()