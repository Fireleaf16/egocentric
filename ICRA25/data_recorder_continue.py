#!/usr/bin/env python3



"""
This script is used for recording the robot states, base trajectory, base to target, arms to target, and controller states.


Author: Yichen Xie


"""

import csv
import os 
import datetime
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64, Bool, Int32, String

from pynput import keyboard

import tf
import sys
import rospy
import numpy as np
## -------------- Class Definition -------------- ##


class DataRecorder():
    def __init__(self):
        ## -------------- Variables -------------- ##
        self.listener = tf.TransformListener()
        self.start_time = rospy.Time.now().to_sec()
        self.origin = "odom"
        self.robot_base_link = "base_footprint"
        self.arm_base_link = "torso_lift_link"
        self.arm_ee_link = ["gripper_left_grasping_frame", "gripper_right_grasping_frame"]
        
        self.object_frames = [["task1_orange_can", "task1_basket_target"],
                              ["task2_lower","task2_upper"],
                              [ "task3_blue","task3_blue_target"],
                              [ "task4_clothes", "task4_candy", "task4_can","task4_clothes_target","task4_candy_target","task4_can_target"]]
        
        self.torso_joint = 0.0
        self.head_joint = [0.0, 0.0]
        self.base_velocity = [0.0, 0.0, 0.0]
        self.control_method = 3
        self.left_robot_activation = 0.0
        self.right_robot_activation = 0.0
        self.right_hand_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.left_hand_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.head_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.right_buttons = {
            'axes': np.array([0.0, 0.0, 0.0]),
            'buttons': np.array([0.0, 0.0, 0.0, 0.0]),
        }
        self.left_buttons = {
            'axes': np.array([0.0, 0.0, 0.0]),
            'buttons': np.array([0.0, 0.0, 0.0, 0.0]),
        }

        self.right_gripper_state = 0
        self.left_gripper_state = 0

        self.left_extend = 0
        self.right_extend = 0
        self.right_forward = 0
        self.left_forward = 0
        self.right_shrink = 0
        self.left_shrink = 0
        self.right_cross = 0
        self.left_cross = 0
        self.left_updown = 0
        self.right_updown = 0


        # collision states
        self.right_target_collision = 0
        self.left_target_collision = 0
        self.right_move_collision = 0
        self.left_move_collision = 0
        self.right_base_fast_domain = 1
        self.right_elbow_collsion = 0
        self.left_elbow_collsion = 0
        self.right_current_above_desk = 0
        self.left_current_above_desk = 0

        self.head_state = 0
        self.obj_idx = 0

        self.key_listener = keyboard.Listener(on_press=self.__on_press)
        self.key_listener.start()
        rospy.sleep(1.5)

        rospy.logwarn("Enter the task number:")

        self.task_num = input("Enter the task number: ")

        rospy.logwarn("Task number: %s", self.task_num)

        rospy.logwarn("Enter the control method: 0 is original, 1 is egocentric, 2 is combined")

        self.control_method = int(input("Enter the control method: 0 is original, 1 is egocentric, 2 is combined"))
        
        rospy.logwarn("Control Method: %d", self.control_method)

        rospy.logwarn("Enter the test_nummber: 1,2,3")

        self.test_number = int(input("Enter the test_number: 1,2,3"))
        
        rospy.logwarn("test_number is: %d", self.test_number)

        self.task_idx = int(self.task_num) - 1
        timestamp = datetime.datetime.now().strftime("%m%d_%H%M%S")

        ## -------------- Publishers -------------- ##
        self.task_num_pub = rospy.Publisher('/Task_Num', Int32, queue_size=1)
        self.timestamp_pub = rospy.Publisher('/Timestamp', String, queue_size=1)
        self.control_method_pub = rospy.Publisher('/Control_method', Int32, queue_size=1)
        self.test_num_pub = rospy.Publisher('/Test_number', Int32, queue_size=1)
        rospy.sleep(0.5)
        self.task_num_pub.publish(int(self.task_num))
        rospy.sleep(0.5)
        self.control_method_pub.publish(self.control_method)
        rospy.sleep(0.5)
        self.test_num_pub.publish(self.test_number)
        rospy.sleep(0.5)
        self.timestamp_pub.publish(timestamp)

        ## -------------- Files -------------- ##
        
        self.controller_states_filename = os.path.expanduser(f"~/ICRA2025/teleop_data/task{self.task_num}/original/test{self.test_number}/controller_states.csv")
        self.egocentric_states_filename = os.path.expanduser(f"~/ICRA2025/teleop_data/task{self.task_num}/egocentric/test{self.test_number}/egocentric_states.csv")
        self.combined_states_filename = os.path.expanduser(f"~/ICRA2025/teleop_data/task{self.task_num}/combined/test{self.test_number}/combined_states.csv")

        if self.control_method == 0:
            filename = "original"
        elif self.control_method == 1:
            filename = "egocentric"
        elif self.control_method == 2:
            filename = "combined"

        self.base_to_target_filename = os.path.expanduser(f"~/ICRA2025/teleop_data/task{self.task_num}/{filename}/test{self.test_number}/robot_to_target/base.csv") 
        self.arm_left_to_target_filename = os.path.expanduser(f"~/ICRA2025/teleop_data/task{self.task_num}/{filename}/test{self.test_number}/robot_to_target/arm_left.csv")
        self.arm_right_to_target_filename = os.path.expanduser(f"~/ICRA2025/teleop_data/task{self.task_num}/{filename}/test{self.test_number}/robot_to_target/arm_right.csv")
        self.robot_states_filename = os.path.expanduser(f"~/ICRA2025/teleop_data/task{self.task_num}/{filename}/test{self.test_number}/robot_states.csv")
        self.base_trajectory_filename = os.path.expanduser(f"~/ICRA2025/teleop_data/task{self.task_num}/{filename}/test{self.test_number}/robot_trajectory.csv")

        ### ---------- Ensure the directory exists --------- ###
        self.__make_sure_path_exists(self.robot_states_filename)
        self.__make_sure_path_exists(self.base_trajectory_filename)
        self.__make_sure_path_exists(self.base_to_target_filename)
        self.__make_sure_path_exists(self.arm_left_to_target_filename)
        self.__make_sure_path_exists(self.arm_right_to_target_filename)
        if self.control_method == 0:
            self.__make_sure_path_exists(self.controller_states_filename)
        elif self.control_method == 1:
            self.__make_sure_path_exists(self.egocentric_states_filename)
        elif self.control_method == 2:
            self.__make_sure_path_exists(self.combined_states_filename)

        ### -------------- Write Headers -------------- ###
        with open(self.robot_states_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time", "left_arm_state","left_gripper_state", "left_arm_x", "left_arm_y", "left_arm_z", "left_arm_qx", "left_arm_qy", "left_arm_qz", "left_arm_qw",
                             "right_arm_state", "right_gripper_state","right_arm_x", "right_arm_y", "right_arm_z", "right_arm_qx", "right_arm_qy", "right_arm_qz", "right_arm_qw",
                            "head_pan", "head_tilt", "torso_lift", "base_linear_x", "base_linear_y", "base_angular_z",
                            "right_target_collision", "left_target_collision", "right_move_collision", "left_move_collision", "base_fast_domain",
                            "right_elbow_collision", "left_elbow_collision", "right_current_above_desk", "left_current_above_desk"])
            
        with open(self.base_trajectory_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time", "base_x", "base_y", "base_z", "base_qx", "base_qy", "base_qz", "base_qw"])

        if self.control_method == 0:
            with open(self.controller_states_filename, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["time", "right_hand_x", "right_hand_y", "right_hand_z", "right_hand_qx", "right_hand_qy", "right_hand_qz", "right_hand_qw",
                                "left_hand_x", "left_hand_y", "left_hand_z", "left_hand_qx", "left_hand_qy", "left_hand_qz", "left_hand_qw",
                                "head_x", "head_y", "head_z", "head_qx", "head_qy", "head_qz", "head_qw",
                                "right_axes_1", "right_axes_2", "right_axes_3", "right_buttons_1", "right_buttons_2", "right_buttons_3", "right_buttons_4",
                                "left_axes_1", "left_axes_2", "left_axes_3", "left_buttons_1", "left_buttons_2", "left_buttons_3", "left_buttons_4"])

        if self.control_method == 1:
            with open(self.egocentric_states_filename, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["time", "right_hand_x", "right_hand_y", "right_hand_z", "right_hand_qx", "right_hand_qy", "right_hand_qz", "right_hand_qw",
                                "left_hand_x", "left_hand_y", "left_hand_z", "left_hand_qx", "left_hand_qy", "left_hand_qz", "left_hand_qw",
                                "head_x", "head_y", "head_z", "head_qx", "head_qy", "head_qz", "head_qw",
                                "right_arm_extend", "right_arm_shrink", "right_arm_forward", "right_arm_updown", "right_arm_cross",
                                "left_arm_extend", "left_arm_shrink", "left_arm_forward", "left_arm_updown", "left_arm_cross",
                                "head_state"])    

        if self.control_method == 2:
            with open(self.combined_states_filename, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["time", "right_hand_x", "right_hand_y", "right_hand_z", "right_hand_qx", "right_hand_qy", "right_hand_qz", "right_hand_qw",
                                "left_hand_x", "left_hand_y", "left_hand_z", "left_hand_qx", "left_hand_qy", "left_hand_qz", "left_hand_qw",
                                "head_x", "head_y", "head_z", "head_qx", "head_qy", "head_qz", "head_qw",
                                "right_axes_1", "right_axes_2", "right_axes_3", "right_buttons_1", "right_buttons_2", "right_buttons_3", "right_buttons_4",
                                "left_axes_1", "left_axes_2", "left_axes_3", "left_buttons_1", "left_buttons_2", "left_buttons_3", "left_buttons_4",
                                "right_arm_extend", "right_arm_shrink", "right_arm_forward", "right_arm_updown", "right_arm_cross",
                                "left_arm_extend", "left_arm_shrink", "left_arm_forward", "left_arm_updown", "left_arm_cross",
                                "head_state"])

        with open(self.base_to_target_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            header = ["time", "obj_idx"]
            for obj in self.object_frames[self.task_idx]:
                header += [f"{obj}_x", f"{obj}_y", f"{obj}_z", f"{obj}_qx", f"{obj}_qy", f"{obj}_qz", f"{obj}_qw"]
            writer.writerow(header)
            
        with open(self.arm_left_to_target_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            header = ["time", "obj_idx"]
            for obj in self.object_frames[self.task_idx]:
                header += [f"{obj}_x", f"{obj}_y", f"{obj}_z", f"{obj}_qx", f"{obj}_qy", f"{obj}_qz", f"{obj}_qw"]
            writer.writerow(header)

        with open(self.arm_right_to_target_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            header = ["time", "obj_idx"]
            for obj in self.object_frames[self.task_idx]:
                header += [f"{obj}_x", f"{obj}_y", f"{obj}_z", f"{obj}_qx", f"{obj}_qy", f"{obj}_qz", f"{obj}_qw"]
            writer.writerow(header)

        
        ## -------------- Subscribers -------------- ##
        rospy.Subscriber('/joint_states', JointState, self.__joint_states_callback)
        rospy.Subscriber('/mobile_base_controller/cmd_vel', Twist, self.__base_callback)
        rospy.Subscriber('/left/robot_activation', Float64, self.__left_robot_activation_callback)
        rospy.Subscriber('/right/robot_activation', Float64, self.__right_robot_activation_callback)
        rospy.Subscriber('/Right_Hand', TransformStamped, self.__right_input_pose_callback)
        rospy.Subscriber('/Right_Buttons', Joy, self.__right_input_buttons_callback)
        rospy.Subscriber('/Left_Hand', TransformStamped, self.__left_input_pose_callback)
        rospy.Subscriber('/Left_Buttons', Joy, self.__left_input_buttons_callback)
        rospy.Subscriber('/Head_Motion', PoseStamped, self.__head_motion_callback)
        rospy.Subscriber('/right/gripper_state', Bool, self.__right_gripper_state_callback)
        rospy.Subscriber('/left/gripper_state', Bool, self.__left_gripper_state_callback)


        ## -------------- Arm state Subscribers ----------- ##
        rospy.Subscriber('/left/arm_extend', Bool, self.__left_arm_extend_callback)
        rospy.Subscriber('/right/arm_extend', Bool, self.__right_arm_extend_callback)

        rospy.Subscriber('/left/arm_shrink', Bool, self.__left_arm_shrink_callback)
        rospy.Subscriber('/right/arm_shrink', Bool, self.__right_arm_shrink_callback)

        rospy.Subscriber('/right/arm_forward', Bool, self.__arm_right_forward_callback)
        rospy.Subscriber('/left/arm_forward', Bool, self.__arm_left_forward_callback)

        rospy.Subscriber('/right/arm_updown', Float64, self.__right_updown_callback)
        rospy.Subscriber('/left/arm_updown', Float64, self.__left_updown_callback)

        rospy.Subscriber('/right/arm_cross', Bool, self.__right_arm_cross_callback)
        rospy.Subscriber('/left/arm_cross', Bool, self.__left_arm_cross_callback)

        ## ------------- Collisions Subscribers ----------- ##
        rospy.Subscriber('right/move_collision', Bool, self.__right_collision_callback)
        rospy.Subscriber('left/move_collision', Bool, self.__left_collision_callback)
        rospy.Subscriber('right/base_fast_domain', Bool, self.__right_base_fast_domain_callback)

        rospy.Subscriber('/right/current_above_desk', Float64, self.__right_current_above_desk_callback)
        rospy.Subscriber('/left/current_above_desk', Float64, self.__left_current_above_desk_callback)

        rospy.Subscriber('/right/target_collision', Bool, self.__right_collsion_callback)
        rospy.Subscriber('/left/target_collision', Bool, self.__left_collsion_callback)

        rospy.Subscriber('right/elbow_collision', Bool, self.__right_elbow_collsion_callback)
        rospy.Subscriber('left/elbow_collision', Bool, self.__left_elbow_collsion_callback)
        ## ------------- Head state Subscribers ----------- ##
        rospy.Subscriber('/head_current_state', Float64, self.__head_callback)
    


    ## -------------- Collision Callbacks -------------- ##
    def __right_collsion_callback(self, msg):
        if msg.data:
            self.right_target_collision = 1
        else:
            self.right_target_collision = 0
    
    def __left_collsion_callback(self, msg):
        if msg.data:
            self.left_target_collision = 1
        else:
            self.left_target_collision = 0 

    def __right_base_fast_domain_callback(self, msg):
        if msg.data:
            self.right_base_fast_domain = 1
        else:
            self.right_base_fast_domain = 0

    def __right_current_above_desk_callback(self, msg):
            self.right_current_above_desk = msg.data
 
    
    def __left_current_above_desk_callback(self, msg):
            self.left_current_above_desk = msg.data

    
    def __right_collision_callback(self, msg):
        if msg.data:
            self.right_move_collision = 1
        else:
            self.right_move_collision = 0
    
    def __left_collision_callback(self, msg):  
        if msg.data:
            self.left_move_collision = 1
        else:
            self.left_move_collision = 0

    def __right_elbow_collsion_callback(self, msg):
        if msg.data:
            self.right_elbow_collsion = 1
        else:
            self.right_elbow_collsion = 0
    
    def __left_elbow_collsion_callback(self, msg): 
        if msg.data:
            self.left_elbow_collsion = 1
        else:
            self.left_elbow_collsion = 0

    ## -------------- Callbacks -------------- ##
    def __on_press(self, key):
        if key == keyboard.Key.space:
            self.obj_idx += 1
            t = rospy.Time.now().to_sec() - self.start_time
            rospy.logwarn("obj_idx: %s, time: %s", self.obj_idx, t)

    def __joint_states_callback(self, msg):

        self.torso_joint = msg.position[20]
        self.head_joint = [msg.position[18], msg.position[19]]

    def __base_callback(self, msg):
        self.base_velocity = [msg.linear.x, msg.linear.y, msg.angular.z]

    def __left_robot_activation_callback(self, msg):
        self.left_robot_activation = msg.data

    def __right_robot_activation_callback(self, msg):
        self.right_robot_activation = msg.data

    def __right_input_pose_callback(self, msg):
        self.right_hand_pose['position'] = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        self.right_hand_pose['orientation'] = np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])

    def __right_input_buttons_callback(self, msg):
        self.right_buttons['axes'] = np.array(msg.axes)
        self.right_buttons['buttons'] = np.array(msg.buttons)

    def __left_input_pose_callback(self, msg):
        self.left_hand_pose['position'] = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        self.left_hand_pose['orientation'] = np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])

    def __left_input_buttons_callback(self, msg):
        self.left_buttons['axes'] = np.array(msg.axes)
        self.left_buttons['buttons'] = np.array(msg.buttons)

    def __head_motion_callback(self, msg):
        self.head_pose['position'] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.head_pose['orientation'] = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    def __right_gripper_state_callback(self, msg):
        if not msg.data:
            self.right_gripper_state = 0
        else:
            self.right_gripper_state = 1

    def __left_gripper_state_callback(self, msg):
        if not msg.data:
            self.left_gripper_state = 0
        else:
            self.left_gripper_state = 1

    ## ------- arm states callback ------ ##

    def __left_arm_extend_callback(self, msg):
        if msg.data:
            self.left_extend = 1
        else:
            self.left_extend = 0

    def __right_arm_extend_callback(self, msg):
        if msg.data:
            self.right_extend = 1
        else:
            self.right_extend = 0

    def __right_arm_cross_callback(self, msg):
        if msg.data:
            self.right_cross = 1
        else:
            self.right_cross = 0

    def __left_arm_cross_callback(self, msg):
        if msg.data:
            self.left_cross = 1
        else:
            self.left_cross = 0

    def __arm_left_forward_callback(self, msg):
        if msg.data:
            self.left_forward = 1
        else:
            self.left_forward = 0

    def __arm_right_forward_callback(self, msg):
        if msg.data:
            self.right_forward = 1
        else:
            self.right_forward = 0

    def __left_arm_shrink_callback(self, msg):
        if msg.data:
            self.left_shrink = 1
        else:
            self.left_shrink = 0
    
    def __right_arm_shrink_callback(self, msg):
        if msg.data:
            self.right_shrink = 1
        else:
            self.right_shrink = 0

    def __right_updown_callback(self, msg):
        if msg.data > 0:
            self.right_updown = 1 # 1 is going up
        else:
            if msg.data < 0:
                self.right_updown = 2 # 2 is going down
            else:
                self.right_updown = 0
    
    def __left_updown_callback(self, msg):
        if msg.data > 0:
            self.left_updown = 1 # 1 is going up
        else:
            if msg.data < 0:
                self.left_updown = 2 # 2 is going down
            else:
                self.left_updown = 0

    ## ------ head states callback ------ ##

    def __head_callback(self, msg):
        self.head_state = msg.data #2.0 rotate left, -2.0 rotate right , 1.0 up, -1.0 down


    ## -------------- Helpers -------------- ##
    def __make_sure_path_exists(self, path):
        directory = os.path.dirname(path)
        if not os.path.exists(directory):
            os.makedirs(directory)


    def __get_arm_transformation(self, side):
        try:
            ee_link = self.arm_ee_link[0] if side == "left" else self.arm_ee_link[1]
            (trans, rot) = self.listener.lookupTransform(self.arm_base_link, ee_link, rospy.Time(0))
            pose_message = Pose()
            pose_message.position.x = trans[0]
            pose_message.position.y = trans[1]
            pose_message.position.z = trans[2]

            pose_message.orientation.x = rot[0]
            pose_message.orientation.y = rot[1]
            pose_message.orientation.z = rot[2]
            pose_message.orientation.w = rot[3]

            return pose_message
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def __get_transformation(self, parent, child):
        try:
            (trans, rot) = self.listener.lookupTransform(parent, child, rospy.Time(0))
            pose_message = Pose()
            pose_message.position.x = trans[0]
            pose_message.position.y = trans[1]
            pose_message.position.z = trans[2]

            pose_message.orientation.x = rot[0]
            pose_message.orientation.y = rot[1]
            pose_message.orientation.z = rot[2]
            pose_message.orientation.w = rot[3]

            return pose_message
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    ## -------------- save to csv -------------- ##

    def robot_states_save_to_csv(self):
        filename = self.robot_states_filename

        left_arm_current_pose = self.__get_arm_transformation("left")
        right_arm_current_pose = self.__get_arm_transformation("right")
        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, self.left_robot_activation, self.left_gripper_state, left_arm_current_pose.position.x, left_arm_current_pose.position.y, left_arm_current_pose.position.z,
                            left_arm_current_pose.orientation.x, left_arm_current_pose.orientation.y, left_arm_current_pose.orientation.z, left_arm_current_pose.orientation.w,
                            self.right_robot_activation, self.right_gripper_state,right_arm_current_pose.position.x, right_arm_current_pose.position.y, right_arm_current_pose.position.z,
                            right_arm_current_pose.orientation.x, right_arm_current_pose.orientation.y, right_arm_current_pose.orientation.z, right_arm_current_pose.orientation.w,
                            self.head_joint[0], self.head_joint[1], self.torso_joint, self.base_velocity[0], self.base_velocity[1], self.base_velocity[2],
                            self.right_target_collision, self.left_target_collision, self.right_move_collision, self.left_move_collision, self.right_base_fast_domain,
                            self.right_elbow_collsion, self.left_elbow_collsion, self.right_current_above_desk, self.left_current_above_desk])
    

    def robot_base_trajectory_save_to_csv(self):
        filename = self.base_trajectory_filename
        base_current_pose = self.__get_transformation(self.origin, self.robot_base_link)
        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, base_current_pose.position.x, base_current_pose.position.y, base_current_pose.position.z,
                            base_current_pose.orientation.x, base_current_pose.orientation.y, base_current_pose.orientation.z, base_current_pose.orientation.w])
    
    def controller_states_save_to_csv(self):
        filename = self.controller_states_filename
        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, self.right_hand_pose['position'][0], self.right_hand_pose['position'][1], self.right_hand_pose['position'][2],
                             self.right_hand_pose['orientation'][0], self.right_hand_pose['orientation'][1], self.right_hand_pose['orientation'][2], self.right_hand_pose['orientation'][3],
                             self.left_hand_pose['position'][0], self.left_hand_pose['position'][1], self.left_hand_pose['position'][2],
                             self.left_hand_pose['orientation'][0], self.left_hand_pose['orientation'][1], self.left_hand_pose['orientation'][2], self.left_hand_pose['orientation'][3],
                             self.head_pose['position'][0], self.head_pose['position'][1], self.head_pose['position'][2],
                             self.head_pose['orientation'][0], self.head_pose['orientation'][1], self.head_pose['orientation'][2], self.head_pose['orientation'][3],
                             self.right_buttons['axes'][0], self.right_buttons['axes'][1], self.right_buttons['axes'][2],
                             self.right_buttons['buttons'][0], self.right_buttons['buttons'][1], self.right_buttons['buttons'][2], self.right_buttons['buttons'][3],
                             self.left_buttons['axes'][0], self.left_buttons['axes'][1], self.left_buttons['axes'][2],
                             self.left_buttons['buttons'][0], self.left_buttons['buttons'][1], self.left_buttons['buttons'][2], self.left_buttons['buttons'][3]])
    
    '''
    "right_arm_extend", "right_arm_shrink", "right_arm_forward", "right_arm_updown", "right_arm_cross",
                             "left_arm_extend", "left_arm_shrink", "left_arm_forward", "left_arm_updown", "left_arm_cross",
                             "head_state"
    '''


    def egocentric_states_save_to_csv(self):
        filename = self.egocentric_states_filename
        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, self.right_hand_pose['position'][0], self.right_hand_pose['position'][1], self.right_hand_pose['position'][2],
                             self.right_hand_pose['orientation'][0], self.right_hand_pose['orientation'][1], self.right_hand_pose['orientation'][2], self.right_hand_pose['orientation'][3],
                             self.left_hand_pose['position'][0], self.left_hand_pose['position'][1], self.left_hand_pose['position'][2],
                             self.left_hand_pose['orientation'][0], self.left_hand_pose['orientation'][1], self.left_hand_pose['orientation'][2], self.left_hand_pose['orientation'][3],
                             self.head_pose['position'][0], self.head_pose['position'][1], self.head_pose['position'][2],
                             self.head_pose['orientation'][0], self.head_pose['orientation'][1], self.head_pose['orientation'][2], self.head_pose['orientation'][3],
                             self.right_extend, self.right_shrink, self.right_forward, self.right_updown, self.right_cross,
                             self.left_extend, self.left_shrink, self.left_forward, self.left_updown, self.left_cross,
                             self.head_state])

    def combined_states_save_to_csv(self):
        filename = self.combined_states_filename
        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, self.right_hand_pose['position'][0], self.right_hand_pose['position'][1], self.right_hand_pose['position'][2],
                             self.right_hand_pose['orientation'][0], self.right_hand_pose['orientation'][1], self.right_hand_pose['orientation'][2], self.right_hand_pose['orientation'][3],
                             self.left_hand_pose['position'][0], self.left_hand_pose['position'][1], self.left_hand_pose['position'][2],
                             self.left_hand_pose['orientation'][0], self.left_hand_pose['orientation'][1], self.left_hand_pose['orientation'][2], self.left_hand_pose['orientation'][3],
                             self.head_pose['position'][0], self.head_pose['position'][1], self.head_pose['position'][2],
                             self.head_pose['orientation'][0], self.head_pose['orientation'][1], self.head_pose['orientation'][2], self.head_pose['orientation'][3],
                             self.right_buttons['axes'][0], self.right_buttons['axes'][1], self.right_buttons['axes'][2],
                             self.right_buttons['buttons'][0], self.right_buttons['buttons'][1], self.right_buttons['buttons'][2], self.right_buttons['buttons'][3],
                             self.left_buttons['axes'][0], self.left_buttons['axes'][1], self.left_buttons['axes'][2],
                             self.left_buttons['buttons'][0], self.left_buttons['buttons'][1], self.left_buttons['buttons'][2], self.left_buttons['buttons'][3],
                             self.right_extend, self.right_shrink, self.right_forward, self.right_updown, self.right_cross,
                             self.left_extend, self.left_shrink, self.left_forward, self.left_updown, self.left_cross,
                             self.head_state])


    def base_to_target_save_to_csv(self):
        filename = self.base_to_target_filename
        base_to_target = []
        for i in range(len(self.object_frames[self.task_idx])):
            #rospy.logwarn("obj: %s", self.object_frames[self.task_idx][i])
            base_to_target.append(self.__get_transformation(self.robot_base_link, self.object_frames[self.task_idx][i]))

        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            content = [t, self.obj_idx]
            for obj in base_to_target:
                content += [obj.position.x, obj.position.y, obj.position.z, obj.orientation.x, obj.orientation.y, obj.orientation.z, obj.orientation.w]
            writer.writerow(content)

    def arm_to_target_save_to_csv(self, side):
        filename = self.arm_left_to_target_filename if side == "left" else self.arm_right_to_target_filename
        arm_link = self.arm_ee_link[0] if side == "left" else self.arm_ee_link[1]
        arm_to_target = []
        for i in range(len(self.object_frames[self.task_idx])):
            arm_to_target.append(self.__get_transformation(arm_link, self.object_frames[self.task_idx][i]))

        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            content = [t, self.obj_idx]
            for obj in arm_to_target:
                content += [obj.position.x, obj.position.y, obj.position.z, obj.orientation.x, obj.orientation.y, obj.orientation.z, obj.orientation.w]
            writer.writerow(content)


    ## -------------- Main Loop -------------- ##
    def run(self):
        self._running = True

        while self._running and not rospy.is_shutdown():

            self.robot_states_save_to_csv()
            self.robot_base_trajectory_save_to_csv()
            self.base_to_target_save_to_csv()
            self.arm_to_target_save_to_csv("left")
            self.arm_to_target_save_to_csv("right")
            #rospy.loginfo(f"control_method: {self.control_method}")
            if self.control_method == 0:
                #rospy.logwarn("recording method0")
                self.controller_states_save_to_csv()
                
            elif self.control_method == 1:
                #rospy.logwarn("recording method1")
                self.egocentric_states_save_to_csv()
            elif self.control_method == 2:
                #rospy.logwarn("recording method2")
                self.combined_states_save_to_csv()
            else:
                continue
                
            rospy.sleep(0.05)

def node_shutdown():
    print("Shutting down data_recorder node")
    

## -------------- Main Function -------------- ## 
def main():
    try:
        rospy.init_node('data_recorder')
        rospy.on_shutdown(node_shutdown)

        app = DataRecorder()
        app.run()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

