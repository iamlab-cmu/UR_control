#!/usr/bin/env python

import math
import rospy
import yaml
import tf2_ros
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
        self.ur5e_arm_pub = rospy.Publisher("/ur_hardware_interface/script_command", String, queue_size=10)
        T.sleep(1)

    def set_tcp(self, tcp_pose):
        set_tcp_string = String()

        set_tcp_string.data =  'set_tcp(p[' + ','.join(str(element) for element in tcp_pose) + '])'

        self.ur5e_arm_pub.publish(set_tcp_string)
        self.ur5e_arm_pub.publish(set_tcp_string)

        T.sleep(1)

        return True

    def home_arm(self, time=None):
        home_joint_angles = [90, -90, 90, -90, -90, -135]

        self.move_joints_in_degrees(home_joint_angles, time)

        return True

    def zero_ft_sensor(self):
        zero_ft_sensor_string = String()

        zero_ft_sensor_string.data = 'zero_ftsensor()'

        self.ur5e_arm_pub.publish(zero_ft_sensor_string)
        self.ur5e_arm_pub.publish(zero_ft_sensor_string)

        T.sleep(1)

        return True

    def end_force_mode(self):
        end_force_mode = String()

        end_force_mode.data = 'end_force_mode()'

        self.ur5e_arm_pub.publish(end_force_mode)
        self.ur5e_arm_pub.publish(end_force_mode)

        T.sleep(1)

        return True

    def force_mode(self, task_frame, selection_vector, wrench, limits, time):
        force_mode_string = String()
        
        force_mode_string.data = 'def force_control():\n    zero_ftsensor()\n    force_mode(p[' + ','.join(str(frame) for frame in task_frame) + '],' +\
                                             '[' + ','.join(str(selection) for selection in selection_vector) + '],' + \
                                             '[' + ','.join(str(force) for force in wrench) + '], 2, ' + \
                                             '[' + ','.join(str(limit) for limit in limits) + '])\n    sleep('+str(time)+')\n    end_force_mode()\nend'

        self.ur5e_arm_pub.publish(force_mode_string)
        self.ur5e_arm_pub.publish(force_mode_string)

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

    def rotate_ee(self, axis, angle, time=None):

        ee_pos = self.get_ee_pose()

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


    #Not working
    def rotate_ee_movec(self, axis, angle):

      ee_pos = self.get_ee_pose()

      axis_angle = [ee_pos[3], ee_pos[4], ee_pos[5]]

      final_axis_angle = utils.rotate_around_axis(axis_angle, axis, angle)

      mid_ee_pos = [ee_pos[0], ee_pos[1], ee_pos[2], 0, 0, 0]

      new_ee_pos = [ee_pos[0], ee_pos[1], ee_pos[2], final_axis_angle[0], final_axis_angle[1], final_axis_angle[2]]

      movec_string = String()

      movec_string.data =  'movec(p[' + ','.join(str(element) for element in mid_ee_pos) + '],'+ \
                                 'p[' + ','.join(str(element) for element in new_ee_pos) + '],a=1.2,v=0.25,r=0.05,mode=0)'
      
      self.ur5e_arm_pub.publish(movec_string)
      self.ur5e_arm_pub.publish(movec_string)

      T.sleep(1)

      return True

    def rotate_ee_degrees(self, axis, angle, time=None):

      return self.rotate_ee(axis, angle / 180 * np.pi)

    def get_ee_pose(self):
        rospy.sleep(1)
        trans = self.tf_buffer.lookup_transform('base', 'tool0_controller', rospy.Time(0))

        orientation_q = trans.transform.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        axis_angle = -utils.quat_to_axis_angle([trans.transform.rotation.x, trans.transform.rotation.y,
                                                trans.transform.rotation.z, trans.transform.rotation.w])

        return [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z,
                axis_angle[0], axis_angle[1], axis_angle[2]]

    def get_joints(self):
        try:
            joint_state_msg = rospy.wait_for_message('/joint_states', JointState, 1)
            return [joint_state_msg.position[0] * 180 / math.pi, joint_state_msg.position[1] * 180 / math.pi, joint_state_msg.position[2] * 180 / math.pi,
                    joint_state_msg.position[3] * 180 / math.pi, joint_state_msg.position[4] * 180 / math.pi, joint_state_msg.position[5] * 180 / math.pi]
        except:
            return None
