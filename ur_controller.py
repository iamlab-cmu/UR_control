#!/usr/bin/env python

import math
import rospy
import yaml
import tf2_ros
import tf_conversions
import numpy as np
import time as T
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from subprocess import call

import utils

class URController:
    def __init__(self):
        rospy.init_node('URController')
        self.current_arm_position = [0.0, 0.0, 0.0]

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.ur5e_arm_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

    def home_arm(self, time=None):
        home_joint_angles = [90, -169, 150, -130, -90, 0]
        move_joints_string = String()

        if time is not None:
            move_joints_string.data =  'movej([' + ','.join(str(angle * np.pi/180.0) for angle in home_joint_angles) + '],t='+str(time)+')'
        else:
            move_joints_string.data =  'movej([' + ','.join(str(angle * np.pi/180.0) for angle in home_joint_angles) + '])'

        self.ur5e_arm_pub.publish(move_joints_string)
        self.ur5e_arm_pub.publish(move_joints_string)

        if time is not None:
            T.sleep(time)
        else:
            T.sleep(1)

        return True

    def force_mode(self, task_frame, selection_vector, wrench, limits, time):
        # force_mode_string = String()



        # with open("force_mode.yaml", 'r') as stream:
        #     try:
        #         msg = yaml.safe_load(stream)
        #         print(msg)
        #     except yaml.YAMLError as exc:
        #         print(exc)

        # force_mode_string.data = 'def testmove():\n' + \
        #                          '    zero_ftsensor()\n' + \
        #                          '    force_mode(p[' + ','.join(str(frame) for frame in task_frame) + '], ' + \
        #                          '[' + ','.join(str(selection) for selection in selection_vector) + '], ' + \
        #                          '[' + ','.join(str(force) for force in wrench) + '], 2, ' + \
        #                          '[' + ','.join(str(limit) for limit in limits) + '])\n' + \
        #                          '    sleep(' + str(time) + ')\n' + \
        #                          '    end_force_mode()\n' + \
        #                          'end\n'

        # print(force_mode_string)

        # self.ur5e_arm_pub.publish(msg)
        # self.ur5e_arm_pub.publish(msg)

        rc = call("./force_mode.sh")

        T.sleep(time)

        return True

    def move_joints_in_degrees(self, desired_arm_angles, time=None):
        move_joints_string = String()

        if time is not None:
            move_joints_string.data =  'movej([' + ','.join(str(angle * np.pi/180.0) for angle in desired_arm_angles) + '],t='+str(time)+')'
        else:
            move_joints_string.data =  'movej([' + ','.join(str(angle * np.pi/180.0) for angle in desired_arm_angles) + '])'

        self.ur5e_arm_pub.publish(move_joints_string)
        self.ur5e_arm_pub.publish(move_joints_string)

        if time is not None:
            T.sleep(time)
        else:
            T.sleep(1)

        return True

    def move_joints_in_radians(self, desired_arm_angles, time=None):
        move_joints_string = String()

        if time is not None:
            move_joints_string.data =  'movej([' + ','.join(str(angle) for angle in desired_arm_angles) + '],t='+str(time)+')'
        else:
            move_joints_string.data =  'movej([' + ','.join(str(angle) for angle in desired_arm_angles) + '])'

        self.ur5e_arm_pub.publish(move_joints_string)
        self.ur5e_arm_pub.publish(move_joints_string)

        if time is not None:
            T.sleep(time)
        else:
            T.sleep(1)

        return True

    def move_ee(self, ee_pose, time=None):
        move_ee_string = String()

        if time is not None:
            move_ee_string.data =  'movel(p[' + ','.join(str(element) for element in ee_pose) + '],t='+str(time)+')'
        else:
            move_ee_string.data =  'movel(p[' + ','.join(str(element) for element in ee_pose) + '])'

        self.ur5e_arm_pub.publish(move_ee_string)
        self.ur5e_arm_pub.publish(move_ee_string)

        if time is not None:
            T.sleep(time)
        else:
            T.sleep(1)

        return True

    ## DANGER: NEEDS MORE TESTING
    def rotate_ee(self, axis, angle, time=None):

      ee_pos = self.get_current_arm_end_effector_position()

      axis_angle = [ee_pos[3], ee_pos[4], ee_pos[5]]

      final_axis_angle = utils.rotate_around_axis(axis_angle, axis, angle)

      new_ee_pos = [ee_pos[0], ee_pos[1], ee_pos[2], final_axis_angle[0], final_axis_angle[1], final_axis_angle[2]]

      move_ee_string = String()

      if time is not None:
          move_ee_string.data =  'movel(p[' + ','.join(str(element) for element in new_ee_pos) + '],t='+str(time)+')'
      else:
          move_ee_string.data =  'movel(p[' + ','.join(str(element) for element in new_ee_pos) + '])'

      self.ur5e_arm_pub.publish(move_ee_string)
      self.ur5e_arm_pub.publish(move_ee_string)

      T.sleep(1)

      return True

    def get_current_arm_end_effector_position(self):
            rospy.sleep(1)
        #try:
            #self.tf_buffer.wait_for_transform('ur_arm_tool0', 'ur_arm_base', rospy.Time(0))
            trans = self.tf_buffer.lookup_transform('ur_arm_base', 'ur_arm_tool0', rospy.Time(0))
            axis_angle = -utils.quat_to_axis_angle([trans.transform.rotation.x, trans.transform.rotation.y,
                                                    trans.transform.rotation.z, trans.transform.rotation.w])

            #euler = tf_conversions.transformations.euler_from_quaternion(trans.transform.rotation)
            return [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z,
                    axis_angle[0], axis_angle[1], axis_angle[2]]
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.sleep(1)
        #     return None

    def get_current_arm_joint_angles(self):
        try:
            joint_state_msg = rospy.wait_for_message('/joint_states', JointState, 1)
            return [joint_state_msg.position[0] * 180 / math.pi, joint_state_msg.position[1] * 180 / math.pi, joint_state_msg.position[2] * 180 / math.pi,
                    joint_state_msg.position[3] * 180 / math.pi, joint_state_msg.position[4] * 180 / math.pi, joint_state_msg.position[5] * 180 / math.pi]
        except:
            return None