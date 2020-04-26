#!/usr/bin/env python
import time
import rospy
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
"""
Topics To Write on:
type: std_msgs/Float64
/qtrobot/pitch_joint_position_controller/command
/qtrobot/roll_joint_position_controller/command
/qtrobot/yaw_joint_position_controller/command
"""

class QtrobotJointMover(object):

    def __init__(self):
        rospy.init_node('jointmover_demo', anonymous=True)
        rospy.loginfo("Qtrobot JointMover Initialising...")
        # self.pub_qtrobot_roll_joint_position = rospy.Publisher('/qtrobot/roll_joint_position_controller/command',
        #                                                     Float64,
        #                                                     queue_size=1)
        self.pub_qtrobot_pitch_joint_position = rospy.Publisher('/qtrobot/pitch_joint_position_controller/command',
                                                             Float64,
                                                             queue_size=1)
        self.pub_qtrobot_yaw_joint_position = rospy.Publisher('/qtrobot/yaw_joint_position_controller/command',
                                                           Float64,
                                                           queue_size=1)
        joint_states_topic_name = "/qtrobot/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.qtrobot_joints_callback)
        qtrobot_joints_data = None
        while qtrobot_joints_data is None:
            try:
                qtrobot_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass

        self.qtrobot_joint_dictionary = dict(zip(qtrobot_joints_data.name, qtrobot_joints_data.position))

    def move_qtrobot_all_joints(self, pitch, yaw):
        # angle_roll = Float64()
        # angle_roll.data = roll
        angle_pitch = Float64()
        angle_pitch.data = pitch
        angle_yaw = Float64()
        angle_yaw.data = yaw
        # self.pub_qtrobot_roll_joint_position.publish(angle_roll)
        self.pub_qtrobot_pitch_joint_position.publish(angle_pitch)
        self.pub_qtrobot_yaw_joint_position.publish(angle_yaw)

    def move_qtrobot_roll_joint(self, position):
        """
        limits radians : lower="-0.2" upper="0.2"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_qtrobot_roll_joint_position.publish(angle)

    def move_qtrobot_pitch_joint(self, position):
        """
        limits radians : lower="0" upper="0.44"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_qtrobot_pitch_joint_position.publish(angle)

    def move_qtrobot_yaw_joint(self, position):
        """
        Limits : continuous, no limits
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_qtrobot_yaw_joint_position.publish(angle)

    def qtrobot_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.qtrobot_joint_dictionary = dict(zip(msg.name, msg.position))

    def qtrobot_check_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'pitch_joint', 'roll_joint', 'yaw_joint' is near the value given
        :param value:
        :return:
        """
        similar = self.qtrobot_joint_dictionary.get(joint_name) >= (value - error ) and self.qtrobot_joint_dictionary.get(joint_name) <= (value + error )

        return similar

    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to positive equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * pi
        mod_angle = int(angle / complete_rev)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * pi

        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def qtrobot_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'pitch_joint', 'roll_joint', 'yaw_joint' is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param value:
        :return:
        """
        joint_reading = self.qtrobot_joint_dictionary.get(joint_name)
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error

        return similar

    def qtrobot_movement_sayno(self):
        """
        Make qtrobot say no with the head
        :return:
        """
        check_rate = 5.0
        position = 0.7


        rate = rospy.Rate(check_rate)
        for repetition in range(2):
            similar = False
            while not similar:
                self.move_qtrobot_yaw_joint(position=position)
                # WARNING: THE COMMAND HAS TO BE PUBLISHED UNTIL THE GOAL IS REACHED
                # This is because, when publishing a topic, the first time doesn't always work.
                similar = self.qtrobot_check_continuous_joint_value(joint_name="HeadYaw", value=position)

                rate.sleep()
            position *= -1



    def qtrobot_movement_look(self, pitch, yaw):
        """
        Make qtrobot look down
        :return:
        """
        check_rate = 5.0
        # position_roll = roll
        position_pitch = pitch
        position_yaw = yaw

        # similar_roll = False
        similar_pitch = False
        similar_yaw = False
        rate = rospy.Rate(check_rate)
        while not (similar_yaw and similar_pitch):
            self.move_qtrobot_all_joints(position_pitch, position_yaw)
            # similar_roll = self.qtrobot_check_continuous_joint_value(joint_name="roll_joint", value=position_roll)
            similar_pitch = self.qtrobot_check_continuous_joint_value(joint_name="HeadPitch", value=position_pitch)
            similar_yaw = self.qtrobot_check_continuous_joint_value(joint_name="HeadYaw", value=position_yaw)
            rate.sleep()


    def qtrobot_lookup(self):

        self.qtrobot_movement_look(pitch=0, yaw=0)

    def qtrobot_lookdown(self):

        self.qtrobot_movement_look(pitch=3.14, yaw=0)

    def qtrobot_lookright(self):

        self.qtrobot_movement_look(pitch=0.0, yaw=3.14)

    def qtrobot_lookleft(self):

        self.qtrobot_movement_look(pitch=0.0, yaw=0)

    # def qtrobot_movement_laugh(self, set_rpy=False, pitch=1.57, yaw=1.57, n_giggle=15):
    def qtrobot_movement_laugh(self, set_rpy=False, pitch=1.57, yaw=0, n_giggle=15):
        """
        Giggle in a given pitch yaw configuration
        :return:
        """
        # position_roll = roll
        position_pitch = pitch
        position_yaw = yaw
        for repetitions in range(n_giggle):
            if set_rpy:
                self.move_qtrobot_all_joints(position_pitch, position_yaw)
            else:
                self.move_qtrobot_yaw_joint(position_yaw)
            time.sleep(0.1)
            position_yaw *= -1

    def qtrobot_moverandomly(self):
        # roll = random.uniform(-0.15, 0.15)
        pitch = random.uniform(-0.15, 0.15)
        # yaw = random.uniform(0.0, 2*pi)
        yaw = random.uniform(0.0, 0.0)
        self.qtrobot_movement_look(pitch, yaw)

    def movement_random_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Start Moving Qtrobot...")

        while not rospy.is_shutdown():
            # for i in range(10):
            #     self.qtrobot_lookup()
            #     self.qtrobot_lookdown()
            for i in range(10):
                self.qtrobot_lookright()
                self.qtrobot_lookleft()

            # self.qtrobot_moverandomly()
            # self.qtrobot_movement_laugh()


if __name__ == "__main__":
    qtrobot_jointmover_object = QtrobotJointMover()
    qtrobot_jointmover_object.movement_random_loop()