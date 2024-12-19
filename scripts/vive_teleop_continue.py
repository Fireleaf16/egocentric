#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This script is used for teleoperating the Tiago++ robots torso, head, and mobile base using HTC vive controllers.

Controller mapping:

    /Head_Motion (HTC vive headset) -> robot head
    /Right_Buttons trackpad -> x, y motion of the mobile base
    /Right_Buttons menu button -> home right arm
    /Right_Buttons squeeze button -> activate right arm
    /Left_Buttons trackpad left / right -> yaw rotation of the mobile base
    /Left_Buttons trackpad up / down -> lift / descend torso by 5 cm
    /Left_Buttons menu button -> home left arm
    /Left_Buttons squeeze button -> activate left arm


Author: Yichen Xie
"""

import sys
import rospy
import math
import numpy as np
import csv
import datetime
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import actionlib
import os
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, Pose
from sensor_msgs.msg import Joy, JointState, LaserScan
from std_msgs.msg import Float64, Bool
from std_msgs.msg import Float32MultiArray

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from pal_startup_msgs.srv import StartupStop, StartupStopRequest, StartupStart, StartupStartRequest
from teleop_tools_msgs.msg import IncrementAction as TTIA
from teleop_tools_msgs.msg import IncrementGoal as TTIG


class JoystickTeleop():
    def __init__(self, sim=True):

        ## -------------- Variables -------------- ##
        self.sim = sim


        ## ------- Limits ------- ##
        self.base_speed = [0.03, 0.1]
        self.rotate_speed = [0.11, 0.31]
        self.torso_limit = 0.07
        self.head_joint_limits = [[-1.24, 1.24], [-0.98, 0.72]]

        ### ------- Head------- ###
        self.alpha = 1.0
        self.initialized = False
        self.filtered_orientation = [0.0, 0.0]  # Initial filtered orientation
        self.prev_head_pos = [0.0, 0.0]
        self.head_orientation = [0.0, 0.0]
        self.head_joint_pitch = 0
        self.head_joint_yaw = 0
        self.head_yaw_threshold = 0.62
        self.head_yaw_duration = 0.001  # seconds
        self.head_yaw_start_time = None
        self.head_pitch_up_threshold = 0.00
        self.head_pitch_down_threshold = -0.85
        self.head_pitch_up_start_time = None
        self.head_pitch_down_start_time = None
        self.head_pitch_duration = 0.1  # seconds
        self.right_forward = False
        self.left_forward = False
        self.head_pitch_up_exceeded = False
        self.head_pitch_down_exceeded = False
        self.head_yaw_exceeded = False 
        self.head_current_state = 0.0

        ### ------- Torso ------- ###
        self.torso_thres = 0.12
        self.torso_joint = 0.05
        self.torso_control_speed = 0.005

        ### ------- Base ------- ###
        self._hz = rospy.get_param('~hz', 10)
        self._forward_rate = self.base_speed[1]
        self._angular = 0
        self._linear = 0
        self._linear_y = 0
        self.angular_rate = 0.5
        self.lidar_warning_range = 1.5
        self.collision_threshold = 0.5
        self.base_domain = True
        ## -------------- Flags -------------- ##
        self.activated = False
        self.gripper_pressed = False
        self.torso_pressed = False
        self.right_extended = False
        self.left_extended = False
        self.right_act = False
        self.left_act = False
        self.left_collision = False
        self.right_collision = False
        self.right_move_collision = False
        self.left_move_collision = False
        self.left_above_desk = 0.0
        self.right_above_desk = 0.0


        ## -------------- Arm State -------------- ##
        self.arm_state = {
            'right_forward': False,
            'left_forward': False,
            'right_extended': False,
            'right_shrink': False,
            'left_extended': False,
            'left_shrink': False,
            'right_cross': False,
            'left_cross': False
        }
        ## -------------- Current control mode -------------- ##
        self._move_mode = 'free'
        self._torso_mode = 'torso'
        ## -------------- ROS Publishers -------------- ##
        if sim:
            self._pub_cmd = rospy.Publisher('key_vel', Twist, queue_size=10)

        else:
            self._pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
            self._pub_move_base = rospy.Publisher('/move_base_target', Float32MultiArray, queue_size=1)

        self.pub_head_state = rospy.Publisher('/head_current_state',Float64,queue_size=1)
        ## -------------- ROS Action Clients ------------ ##
        self.torso_client = actionlib.SimpleActionClient('/torso_controller/increment', TTIA)
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=2)
        self.torso_pub = rospy.Publisher('/torso_state', Float64, queue_size=1)
        rospy.sleep(0.1)

        ## -------------- ROS Subscribers -------------- ##

        # rospy.Subscriber('/Right_Hand', TransformStamped, self.__right_input_pose_callback)
        rospy.Subscriber('/Right_Buttons', Joy, self.__right_input_buttons_callback)
        # rospy.Subscriber('/Left_Hand', TransformStamped, self.__left_input_pose_callback)
        rospy.Subscriber('/Left_Buttons', Joy, self.__left_input_buttons_callback)
        rospy.Subscriber('/Head_Motion', PoseStamped, self.__head_motion_callback)
        rospy.Subscriber('/joint_states', JointState, self.__joint_states_callback)
        rospy.Subscriber('/left/arm_extend', Bool, self.__left_arm_extend_callback)
        rospy.Subscriber('/right/arm_extend', Bool, self.__right_arm_extend_callback)
        rospy.Subscriber('/left/arm_shrink', Bool, self.__left_arm_shrink_callback)
        rospy.Subscriber('/right/arm_shrink', Bool, self.__right_arm_shrink_callback)
        rospy.Subscriber('/left/arm_updown', Float64, self.__left_arm_updown_callback)
        rospy.Subscriber('/right/arm_updown', Float64, self.__right_arm_updown_callback)
        rospy.Subscriber('/right/arm_forward', Bool, self.__right_arm_forward_callback)
        rospy.Subscriber('/left/arm_forward', Bool, self.__left_arm_forward_callback)
        rospy.Subscriber("/scan", LaserScan, self.__scan_callback)
        rospy.Subscriber('/right/robot_activation', Float64, self.__right_activation_callback, queue_size=1)
        rospy.Subscriber('/left/robot_activation', Float64, self.__left_activation_callback, queue_size=1)
        rospy.Subscriber('/right/arm_collision', Bool, self.__right_arm_collision_callback)
        rospy.Subscriber('/left/arm_collision', Bool, self.__left_arm_collision_callback)
        rospy.Subscriber('/right/arm_cross', Bool, self.__right_arm_cross_callback)
        rospy.Subscriber('/left/arm_cross', Bool, self.__left_arm_cross_callback)
        rospy.Subscriber('right/base_fast_domain', Bool, self.__right_base_fast_domain_callback)
        rospy.Subscriber('/right/current_above_desk', Float64, self.__right_current_above_task_callback)
        rospy.Subscriber('/left/current_above_desk', Float64, self.__left_current_above_task_callback)
        rospy.Subscriber('right/move_collision', Bool, self.__right_collision_callback)
        rospy.Subscriber('left/move_collision', Bool, self.__left_collision_callback)

        rospy.sleep(0.1)


        ## -------------- Shut Down Head Manager --------------

        if not sim:

            rospy.wait_for_service('/pal_startup_control/stop')
            try:
                stop_service = rospy.ServiceProxy('/pal_startup_control/stop', StartupStop)
                request = StartupStopRequest()
                request.app = 'head_manager'
                response = stop_service(request)

            except rospy.ServiceException as e:
                print("Service call failed:", e)


        rospy.logwarn("\n\n------------vive teleop ready------------\n\n")

    ## -------------- Callback Functions -------------- ## 
    def __scan_callback(self, data):
        # Store the received scan data
        self.scan_data = data
        if self.scan_data is not None:
            # Convert scan data to numpy array
            self.ranges = np.array(self.scan_data.ranges)
    
    def __right_current_above_task_callback(self, msg):
        self.right_above_desk = msg.data
    
    def __left_current_above_task_callback(self, msg):
        self.left_above_desk = msg.data

    def __right_base_fast_domain_callback(self, msg):
        if msg.data:
            self._forward_rate = self.base_speed[1]
            self.base_domain = True
        else:
            self._forward_rate = self.base_speed[0]
            self.base_domain = False

    def __right_collision_callback(self, msg):
        if msg.data == True:
            self.right_move_collision = True
        else:
            self.right_move_collision = False
    
    def __left_collision_callback(self, msg):
        if msg.data == True:
            self.left_move_collision = True
        else:
            self.left_move_collision = False


    def __right_activation_callback(self, msg):
        if msg.data == 2.0 and not self.right_act:
            self.right_act = True

        if msg.data == 0.0 and self.right_act:
            self.right_act = False


    def __left_activation_callback(self, msg):
        if msg.data == 2.0 and not self.left_act:
            self.left_act = True

        if msg.data == 0.0 and self.left_act:
            self.left_act = False


    def __left_input_buttons_callback(self, msg):
        """
            determine if it is squeezing or button press 

        """
        # if msg.buttons[0] == 1:
        #     self._torso_increment(0.05)  # Increment for torso up

        if (msg.buttons[2] == 1.0) and (not msg.axes[0] == 0.0 or not msg.axes[1] == 0.0):
            self._base_control(msg)
            self._move_mode = 'base'
        else:
            self._linear = 0.0
            self._linear_y = 0.0
            self._move_mode = 'free'
            # self._publish()


    def __right_input_buttons_callback(self, msg):
        """
            control mobile base using the trackpad on the left controller

        """
        if (msg.buttons[2] == 1.0) and (abs(msg.axes[0]) > 0.1 or abs(msg.axes[1]) > 0.1):
            if(abs(msg.axes[0]) > abs(msg.axes[1]) + 0.2):
                self._base_rotate(msg)
                self._move_mode = 'base'
            elif(abs(msg.axes[1]) > abs(msg.axes[0]) + 0.2):
                if msg.buttons[2] == 1.0:
                    self.torso_pressed = True
                    self._torso_increment(np.sign(msg.axes[1]) * self.torso_control_speed)  # Increment for torso up
                    self._torso_mode = 'torso'
                    # rospy.sleep(0.2)
                # if msg.buttons[2] == 0.0:
        else:
            self.torso_pressed = False
            self._torso_mode = 'free'
            if self._move_mode != 'head':
                self._move_mode = 'free'
                self._angular = 0.0
        ## menu button pressed -> home 
        # if msg.buttons[0] == 1:
        #     self,home_robot_arms()
            # self._torso_increment(-0.05)  # Increment for torso up

    def __joint_states_callback(self, msg):

        self.torso_joint = msg.position[20]
        self.head_joint_pitch = msg.position[19]                                                                            
        self.head_joint_yaw = msg.position[18]

            #rospy.logwarn("torso extended, base slow down")

    ## -------------- Arm Control Movement-------------- ##
    def __update_arm_state(self, arm, state):
        self.arm_state[arm] = state
        if self._move_mode != 'base':
            if (not self.right_collision) and (not self.left_collision):
                self.__apply_movement()

    def __right_arm_collision_callback(self, msg):
            self.right_collision = msg.data

    def __left_arm_collision_callback(self, msg):
            self.left_collision = msg.data


    def __right_arm_forward_callback(self, msg):
        self.__update_arm_state('right_forward', msg.data)

    def __left_arm_forward_callback(self, msg):
        self.__update_arm_state('left_forward', msg.data)

    def __right_arm_extend_callback(self, msg):
        self.__update_arm_state('right_extended', msg.data)

    def __right_arm_shrink_callback(self, msg):
        self.__update_arm_state('right_shrink', msg.data)

    def __left_arm_extend_callback(self, msg):
        self.__update_arm_state('left_extended', msg.data)

    def __left_arm_shrink_callback(self, msg):
        self.__update_arm_state('left_shrink', msg.data)


    def __right_arm_cross_callback(self, msg):
        self.__update_arm_state('right_cross', msg.data)
    
    def __left_arm_cross_callback(self, msg):
        self.__update_arm_state('left_cross', msg.data)



    def __left_arm_updown_callback(self, msg):
        if msg.data > 0 and self._torso_mode == 'free' and (self.left_above_desk != 2.0):
            self.left_updown = self.torso_control_speed
            self._torso_increment(self.left_updown)
            #rospy.logwarn("left arm controlled torso moving up")
        elif msg.data < 0 and self._torso_mode == 'free' and (self.left_above_desk == 0.0):
                self.left_updown = -self.torso_control_speed
                self._torso_increment(self.left_updown)
                #rospy.logwarn("left arm controlled torso moving down")
        else:
            self.left_updown = 0.0

    def __right_arm_updown_callback(self, msg):
        if msg.data > 0 and self._torso_mode == 'free' and (self.right_above_desk != 2.0):
            self.right_updown = self.torso_control_speed
            self._torso_increment(self.right_updown)
            #rospy.logwarn("right arm controlled torso moving up")
        elif msg.data < 0 and self._torso_mode == 'free' and (self.right_above_desk == 0.0):
            self.right_updown = -self.torso_control_speed
            self._torso_increment(self.right_updown)
            #rospy.logwarn("right arm controlled torso moving down")
        else:
            self.right_updown = 0.0


    def __head_motion_callback(self, msg):
        """
            control mobile base using the trackpad on the left controller

        """

        if not self.initialized:
            self.initialized = True
            self.filtered_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]


        # Apply low-pass filter for head motion
        filtered_orientation_x = self.alpha * msg.pose.orientation.x + (1 - self.alpha) * self.filtered_orientation[0]
        filtered_orientation_y = self.alpha * msg.pose.orientation.y + (1 - self.alpha) * self.filtered_orientation[1]
        filtered_orientation_z = self.alpha * msg.pose.orientation.z + (1 - self.alpha) * self.filtered_orientation[2]
        filtered_orientation_w = self.alpha * msg.pose.orientation.w + (1 - self.alpha) * self.filtered_orientation[3]

        roll, pitch, yaw = euler_from_quaternion([filtered_orientation_x, filtered_orientation_y, filtered_orientation_z, filtered_orientation_w])
        # roll, pitch, yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])


        self.head_orientation = [round(self.bound(yaw, self.head_joint_limits[0][0], self.head_joint_limits[0][1]), 2),
                            round(self.bound(-pitch-0.52, self.head_joint_limits[1][0], self.head_joint_limits[1][1]), 2)]

        self.filtered_orientation = [filtered_orientation_x, filtered_orientation_y, filtered_orientation_z, filtered_orientation_w]
        # rospy.logwarn("head orientation = %s, %s",round(math.degrees(head_orientation[0])), round(math.degrees(head_orientation[1])))

        # self._send_head_goal(head_orientation)

        ## -------------- Head Control movement-------------- ##
        current_time = rospy.Time.now()

        head_flag = True
        # Move torso up if head pitch exceeds the up threshold for the specified duration
        if self.head_joint_pitch > self.head_pitch_up_threshold and self.left_above_desk != 2.0 and self.right_above_desk != 2.0:
            if self._torso_mode != 'torso':
                    self._torso_increment(self.torso_control_speed)  # Move torso up continuously
                    #rospy.logwarn("head keep looking up torso moving up")
                    self.head_current_state = 1.0
                    self.pub_head_state.publish(self.head_current_state)
                    head_flag = False
                    self._torso_mode = 'head'


        # Move torso down if head pitch exceeds the down threshold for the specified duration
        if self.head_joint_pitch < self.head_pitch_down_threshold and self.left_above_desk == 0.0 and self.right_above_desk == 0.0:
            if self._torso_mode != 'torso':
                    self._torso_increment(-self.torso_control_speed)  # Move torso down continuously
                    #rospy.logwarn("head keep looking down torso moving down")
                    self.head_current_state = -1.0
                    self.pub_head_state.publish(self.head_current_state)
                    head_flag = False 
                    self._torso_mode = 'head'


        
        if self.head_joint_yaw > self.head_yaw_threshold:
            if self._move_mode != 'base':
                if self.base_domain:
                    self._angular = self.rotate_speed[1]  # Rotate left continuously
                else:
                    self._angular = self.rotate_speed[0]
                #rospy.logwarn("head keep looking left rotate left")
                self.head_current_state = 2.0
                self.pub_head_state.publish(self.head_current_state)
                head_flag = False
                self._move_mode = 'head'
        elif self.head_joint_yaw < -self.head_yaw_threshold:
            if self._move_mode != 'base':
                if self.base_domain:
                    self._angular = -self.rotate_speed[1] # Rotate right continuously
                else:
                    self._angular = -self.rotate_speed[0]
                #rospy.logwarn("head keep looking right rotate right")
                self.head_current_state = -2.0
                self.pub_head_state.publish(self.head_current_state)
                head_flag = False                    
                self._move_mode = 'head'
        else:
            if self._move_mode != 'base':
                self._angular = 0.0  # Stop rotating if within threshold
                self._move_mode = 'free'
            #self.head_current_state = 0.0
        
        if head_flag:
            self.pub_head_state.publish(0.0)
            if self._torso_mode != 'torso':
                self._torso_mode = 'free'


    ## -------------- Helper Functions -------------- ## 

    def bound(self, low, high, value):
         return max(low, min(high, value))

    def _base_control(self, joy_msg):
        linear = joy_msg.axes[1]
        linear_y = -joy_msg.axes[0]
        angle_dir = math.atan2(linear_y, linear)
        linear = np.cos(angle_dir) * self._forward_rate
        linear_y = np.sin(angle_dir) * self._forward_rate
        for i in range(0, len(self.ranges), 3):
            distance = self.ranges[i]
            if distance < self.lidar_warning_range:
                # print(self.scan_data.angle_min)
                angle =  self.scan_data.angle_increment * i
                x = distance * np.sin(angle)
                y = distance * np.cos(angle)
                # print(x, y)
                if distance < self.collision_threshold:
                    if abs(x) > abs(y):
                        if linear_y * x < 0:
                            linear_y = 0.0
                    elif abs(y) >= abs(x):
                        if linear * y < 0:
                            linear = 0.0
                    # rospy.logwarn("collision detected: %s, (x, y) = %s, %s", distance, x, y)
                    # break
        array_data = Float32MultiArray()
        array_data.data = [linear, linear_y]
        self._pub_move_base.publish(array_data)
        if (not self.right_move_collision) and (not self.left_move_collision):
            self._linear = linear
            self._linear_y = linear_y
        else:
            self._linear = 0.0
            self._linear_y = 0.0

    def _base_rotate(self, msg):
        if self.base_domain:
            angular = -np.sign(msg.axes[0])*self.rotate_speed[1]
        else:
            angular = -np.sign(msg.axes[0])*self.rotate_speed[0]
        self._angular = angular
    


    def _send_head_goal(self, head_position):

        if not ((abs(self.prev_head_pos[0] - head_position[0]) <= 0.01) and (abs(self.prev_head_pos[1] - head_position[1]) <= 0.01)):

            # Create a trajectory point with the adjusted head position
            point = JointTrajectoryPoint()
            point.positions = head_position
            point.velocities = [abs(self.prev_head_pos[0] - head_position[0]), abs(self.prev_head_pos[1] - head_position[1])]  # Set desired joint velocities
            point.time_from_start = rospy.Duration(0.8)  # A small duration for smooth motion

            # Create and publish the head pose
            head_goal = JointTrajectory()
            head_goal.joint_names = ["head_1_joint", "head_2_joint"]
            head_goal.points = [point]
            head_goal.header.stamp = rospy.Time.now()
            self.head_pub.publish(head_goal)

        else:
            pass

        self.prev_head_pos = head_position

    def _torso_increment(self, increment_value):
        goal = TTIG()
        goal.increment_by = [increment_value]
        if self.torso_joint >= self.torso_limit:

            self.torso_client.send_goal(goal)
        else:
            #rospy.logwarn("Torso: lower limit")
            if increment_value > 0.0:
                self.torso_client.send_goal(goal)


    def __apply_movement(self):
        if self.base_domain:
            self._forward_rate = self.base_speed[1]
        else:
            self._forward_rate = self.base_speed[0]
        if (self.arm_state['right_shrink'] and self.right_act )or (self.arm_state['left_shrink'] and self.left_act):            
            self.__move_base_and_torso((-1, 0, 0))
        elif (self.arm_state['right_forward'] and self.right_act)or (self.arm_state['left_forward'] and self.left_act):
            self.__move_base_and_torso((1, 0, 0))
        elif (self.arm_state['right_extended'] and self.right_act) or (self.arm_state['left_cross'] and self.left_act):
            self.__move_base_and_torso((0, -1, 0))
        elif (self.arm_state['left_extended'] and self.left_act) or (self.arm_state['right_cross'] and self.right_act):
            self.__move_base_and_torso((0, 1, 0))
        else:
            self.__move_base_and_torso(None)

    def __move_base_and_torso(self, translation):
        if translation is not None:
            linear_x = translation[0] * self._forward_rate
            linear_y = translation[1] * self._forward_rate
            for i in range(0, len(self.ranges), 3):
                distance = self.ranges[i]
                if distance < self.lidar_warning_range:
                    # print(self.scan_data.angle_min)
                    angle =  self.scan_data.angle_increment * i
                    x = distance * np.sin(angle)
                    y = distance * np.cos(angle)
                    # print(x, y)
                    if distance < self.collision_threshold:
                        if abs(x) > abs(y):
                            if linear_y * x < 0:
                                linear_y = 0.0
                        elif abs(y) >= abs(x):
                            if linear_x * y < 0:
                                linear_x = 0.0
            self._linear = linear_x
            self._linear_y = linear_y
            
        elif self._move_mode != 'base':
                self._linear = 0
                self._linear_y = 0
                #self._forward_rate = self.base_speed[1]
                


    def _get_twist(self, linear, linear_y, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.linear.y = linear_y
        twist.angular.z = angular
        return twist

    def _publish(self):

        twist = self._get_twist(self._linear, self._linear_y, self._angular)
        self._pub_cmd.publish(twist)
        self._send_head_goal(self.head_orientation)





    def run(self):
        # rate = rospy.Rate(self._hz)
        self._running = True

        while self._running and not rospy.is_shutdown():
            # rospy.logwarn("%s", self.__get_arm_left_transformation())
            if self.base_domain:
                self._forward_rate = self.base_speed[1]
                self.head_yaw_threshold = 0.62
            else:
                self.head_yaw_threshold = 0.62

            self._publish()
            #rospy.logwarn(f"current_speed is:{self._forward_rate}" )
            #rospy.logwarn(f"current_mode is:{self._move_mode}" )
            rospy.sleep(0.01)
            # rate.sleep()



## -------------- Node Destructor -------------- ## 

def node_shutdown():
    rospy.wait_for_service('/pal_startup_control/start')
    try:
        start_service = rospy.ServiceProxy('/pal_startup_control/start', StartupStart)
        request = StartupStartRequest()
        request.app = 'head_manager'
        response = start_service(request)
        # rospy.logwarn("%s", request)
        # rospy.logwarn("%s", response)

        print('\nhead_manager started back up\n')
    except rospy.ServiceException as e:
        print('\nFailed to start head_manager\n')



def node_shutdown_sim():
    print('\nvive_teleop has been shutdown\n')


## -------------- Main Function -------------- ## 

def main():
    try:
        rospy.init_node('vive_teleop')

        args = rospy.myargv(argv=sys.argv)
        sim = args[1]

        if sim == "true":
            sim_param = True
            rospy.on_shutdown(node_shutdown_sim)
        elif sim == "false":
            sim_param = False
            rospy.on_shutdown(node_shutdown)

        app = JoystickTeleop(sim=sim_param)
        app.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

