#!/usr/bin/env python

import rospy
import tf
from tf import transformations
import numpy as np
from std_msgs.msg import Float32, Int8

#Rotate a vector by a quaternion
def qv_mult(q, v):
    v = v + (0.0,)
    q_conj = transformations.quaternion_conjugate(q)
    return transformations.quaternion_multiply(transformations.quaternion_multiply(q,v), q_conj)[:-1]

def wrap_to_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

#Gets the angle between two vectors
def angle_between_vecs(v1, v2):
    return np.arccos(np.clip(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)),-1, 1))

# Gets the angle between a vector u, and a plane defined by normal vector n
def angle_from_vector_to_plane(u, n):
    return wrap_to_pi(np.pi / 2 - angle_between_vecs(u, n))

# Gets the angle between two quaternions for a given axis vector
def angle_between_quats(q1, q2, v):
    v1 = qv_mult(q1, v)
    v2 = qv_mult(q2, v)
    return angle_between_vecs(v1,v2)

# Add support for fake enums
def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['to_string'] = reverse
    return type('Enum', (), enums)

class RulaNode():

    def __init__(self):
        self.node_name = "rula_node"
        rospy.loginfo("[%s] initialized", self.node_name)

        # Constants
        self.transform_timeout = 1  # The number of seconds to wait for a transform to timeout before giving up
        # Setup publishers
        rula_angle_names = []
        self.RulaJoints = enum("right_upper", "left_upper", "right_lower", "left_lower", "neck", "trunk", "right_shoulder", "left_shoulder")
        self.SkeletonFrames = enum("head", "shoulder_center", "elbow_right", "elbow_left", "wrist_right", "wrist_left", "base_link", "shoulder_right", "shoulder_left")
        rj = self.RulaJoints
        sf = self.SkeletonFrames

        pub_joints = []
        pub_scores = []
        for name in rj.to_string.itervalues():
            pub_joints.append(rospy.Publisher("~" + name, Float32, queue_size=10))
            pub_scores.append(rospy.Publisher("~scores/" + name, Int8, queue_size=10))
        pub_scores.append(rospy.Publisher("~scores/total", Int8,queue_size=10 ))
        score_msg = Int8()
        joint_msg = Float32()

        self.listener = tf.TransformListener()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Listen for the tf frames
            q=[None] * len(sf.to_string)
            for id, frame in sf.to_string.iteritems():
                q[id] = self.getJointQuat('/skeleton_frame', frame)

            # For shoulders, the relative position is used instead of relative angle
            q[sf.shoulder_right] = self.getJointPos('/shoulder_center', 'shoulder_left')
            q[sf.shoulder_left] = self.getJointPos('/shoulder_center', 'shoulder_right')

            # Go around again if transforms are missing
            if any(x is None for x in q):
                rospy.logwarn("Transforms were missing. Waiting for transforms...")
                rospy.sleep(1)
                continue

            # Listen for joint transforms
            rula_angles=[0] * len(rj.to_string)
            rula_angles[rj.right_upper] = angle_from_vector_to_plane(qv_mult(q[sf.elbow_right],(0,1,0)),qv_mult(q[sf.base_link], (-1,0,0)))
            rula_angles[rj.left_upper] = angle_from_vector_to_plane(qv_mult(q[sf.elbow_left],(0,1,0)),qv_mult(q[sf.base_link], (-1,0,0)))
            rula_angles[rj.right_lower] = angle_between_quats(q[sf.wrist_right], q[sf.elbow_right], (0,1,0))
            rula_angles[rj.left_lower] = angle_between_quats(q[sf.wrist_left], q[sf.elbow_left], (0,1,0))
            rula_angles[rj.neck] = angle_between_quats(q[sf.head], q[sf.shoulder_center], (0,1,0))
            rula_angles[rj.trunk] = angle_from_vector_to_plane(qv_mult(q[sf.head], (0,1,0)), qv_mult(q[sf.base_link], (-1,0,0)))
            rula_angles[rj.right_shoulder] = q[sf.shoulder_right][1]
            rula_angles[rj.left_shoulder] = q[sf.shoulder_left][1]

            scores = self.calcRula(rula_angles)
            # Publish joint angles and scores
            for id, name in rj.to_string.iteritems():
                joint_msg.data = np.rad2deg(rula_angles[id])
                pub_joints[id].publish(joint_msg)
                score_msg.data = scores[id]
                pub_scores[id].publish(score_msg)

            # Publish the total rula
            score_msg.data = scores[-1]
            pub_scores[-1].publish(score_msg)
            rate.sleep()

        # #Test1
        # v1=np.array([5,0,0])
        # v2=np.array([0,0,2])
        # rospy.loginfo("[angle_between]Expected output: 90 deg. Actual output: %s", angle_between(v1,v2))
        # rospy.loginfo("[wrap_to_pi]Expected output: %s. Actual output: %s", 0, wrap_to_pi(4*np.pi))
        # rospy.loginfo("[angle_from_vector_to_plane]Expected output: 0. Actual output: %s", angle_from_vector_to_plane(v1,v2))
        # v2=np.array([5,5,5])
        # rospy.loginfo("[angle_between]Expected output: 0.955. Actual output: %s", angle_between(v1,v2))
        # rospy.loginfo("[angle_from_vector_to_plane]Expected output: 0.615. Actual ouptut: %s", angle_from_vector_to_plane(v2,v1))
        # rospy.loginfo("[qv_mult]Expected Value: (2,0,0). Actual Value: %s", qv_mult(transformations.quaternion_about_axis(np.pi/2,(0,1,0)),(0,0,2)))

    # Wait for and receive transform
    def getJointQuat(self,target, source):
        try:
            self.listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(self.transform_timeout))
            q = self.listener.lookupTransform(target, source, rospy.Time())[1]

        except:
            q = None
        return q

    # Wait for and receive transform
    def getJointPos(self,target, source):
        try:
            self.listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(self.transform_timeout))
            p = self.listener.lookupTransform(target, source, rospy.Time())[0]

        except:
            p = None
        return p

    # Calculate RULA score from joint angles returns an array of subscores with the total as the last element
    def calcRula(self, rula_angles):
        rj = self.RulaJoints
        scores = [0]*len(rj.to_string)
        scores[rj.right_upper] = self.cutpoints(np.rad2deg(rula_angles[rj.right_upper]), np.array([-20, 20, 45, 90]), np.array([2, 1, 2, 3, 4]))
        scores[rj.left_upper] = self.cutpoints(np.rad2deg(rula_angles[rj.left_upper]), np.array([-20, 20, 45, 90]), np.array([2, 1, 2, 3, 4]))
        scores[rj.right_lower] = self.cutpoints(np.rad2deg(rula_angles[rj.right_lower]), np.array([50, 100]), np.array([2, 1, 2]))
        scores[rj.left_lower] = self.cutpoints(np.rad2deg(rula_angles[rj.left_lower]), np.array([50, 100]), np.array([2, 1, 2]))
        scores[rj.neck] = self.cutpoints(np.rad2deg(rula_angles[rj.neck]), np.array([0, 20, 30]), np.array([4, 1, 2, 3]))
        scores[rj.trunk] = self.cutpoints(np.rad2deg(rula_angles[rj.trunk]), np.array([5, 20, 60]), np.array([1, 2, 3, 4]))
        scores[rj.right_shoulder] = self.cutpoints(rula_angles[rj.right_shoulder], np.array([-0.08]), np.array([0,1]))
        scores[rj.left_shoulder] = self.cutpoints(rula_angles[rj.left_shoulder], np.array([-0.08]), np.array([0,1]))

        tableA = np.array([[1,2,2],[2,3,3],[3,3,4],[4,4,4],[5,5,6],[7,8,9]])
        upper =  max(scores[rj.left_upper],scores[rj.right_upper])
        lower = max(scores[rj.right_lower], scores[rj.left_lower])
        upper += min(scores[rj.left_shoulder], scores[rj.right_shoulder]) #Take into account raised shoulders
        scoreA = tableA[upper-1, lower-1]


        tableB = np.array([[1,2,3,5,6,7], [2,2,4,5,6,7], [3,3,4,5,6,7], [5,5,6,7,7,8], [7,7,7,8,8,8], [8,8,8,8,9,9]])
        scoreB = tableB[scores[rj.neck]-1, scores[rj.trunk]-1]

        tableC = np.array([[1,2,3,3,4,5,5,5,5], [2,2,3,4,4,5,5,5,5], [3,3,3,4,4,5,6,6,6],
                           [3,3,3,4,5,6,6,6,6], [4,4,4,5,6,7,7,7,7], [4,4,5,6,6,7,7,7,7],
                           [5,5,6,6,7,7,7,7,7], [5,5,6,7,7,7,7,7,7], [5,5,6,7,7,7,7,7,7]])

        scoreC = tableC[scoreA-1, scoreB-1]
        scores.append(scoreC)
        return scores

    # Finds the value in values, where x lies between the values in cutpoints
    # cutpoints and values should both be 1D numpy arrays where cutpoints has one fewer value than values
    def cutpoints(self, x, cutpoints, values):
        cutpoints = np.append(cutpoints, float('inf'))
        return values[x<cutpoints][0]

    # Shutdown
    def onShutdown(self):
        rospy.loginfo("[%s] Shutting down.", self.node_name)

if __name__ == '__main__':
    rospy.init_node("rula_node", anonymous=False)
    try:
        rula_node = RulaNode()
        rospy.on_shutdown(rula_node.onShutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass