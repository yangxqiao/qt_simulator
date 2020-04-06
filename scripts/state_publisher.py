#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import roslib
import tf
from std_msgs.msg import Header

if __name__ == '__main__':

    broadcaster = tf.TransformBroadcaster()
    joint_pub = rospy.Publisher('joint_state', JointState, queue_size=10)
    rospy.init_node('state_publisher', anonymous=True)
    rate = rospy.Rate(30)  # 10hz

    odom_trans = geometry_msgs.msg.TransformStamped()
    odom_trans.header.frame_id = 'odom'
    odom_trans.child_frame_id = 'base_link'

    joint_state = JointState()

    right_shoulder_pitch = 0
    right_shoulder_roll = 0
    right_elbow_roll = -0.5
    step_size_RSP = 0.02
    step_size_RSR = 0.02
    step_size_RER = 0.02
    state = 0
    rospy.sleep(5)

    while not rospy.is_shutdown():
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()

        joint_state.name = ['HeadYaw', 'HeadPitch', 'RightShoulderPitch', 'RightShoulderRoll', 'RightElbowRoll',
                            'LeftShoulderPitch', 'LeftShoulderRoll', 'LeftElbowRoll']
        joint_state.position = [0, 0, right_shoulder_pitch, right_shoulder_roll, right_elbow_roll, 1.14, -0.95, -0.87]

        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.transform.translation.x = 0
        odom_trans.transform.translation.y = 0
        odom_trans.transform.translation.z = 0

        odom_trans.transform.rotation.x = 0
        odom_trans.transform.rotation.y = 0
        odom_trans.transform.rotation.z = 0
        odom_trans.transform.rotation.w = 0

        joint_pub.publish(joint_state)
        broadcaster.sendTransformMessage(odom_trans)
        # broadcaster.sendTransform((0, 0, 0), (0, 0, 0), rospy.Time.now(),  'base_link', 'odom')

        if state is 0:
            right_elbow_roll -= step_size_RER
            if right_elbow_roll < -1.25:
                state = 1
        elif state is 1:
            right_shoulder_pitch += step_size_RSP
            if right_shoulder_pitch > 1.58:
                state = 2
        elif state is 2:
            right_elbow_roll += step_size_RER
            if right_elbow_roll < -1.57 or right_elbow_roll > -1:
                step_size_RER *= -1

        rate.sleep()