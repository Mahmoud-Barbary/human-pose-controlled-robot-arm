#!/usr/bin/env python3

# Software License Agreement (BSD License)
# (C) SRI International and contributors

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot
# Modified by Michael Leiter & Mahmoud Barbary for CSCI 5551 Project

# Description:
# This script uses MediaPipe's pose estimation from a webcam feed to track a human's arm
# and control a Kinova Gen3 robot arm accordingly via MoveIt in ROS. Specifically, it calculates
# angles at the shoulder and elbow joints and maps them to the robot's arm joints.

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import numpy as np
import cv2
import mediapipe as mp

class SetAngleMoveItTrajectories(object):
    """
    Initializes the MoveIt interface and controls the Kinova Gen3 arm to follow given joint angles.
    """
    def __init__(self):
        super(SetAngleMoveItTrajectories, self).__init__()

        # Initialize ROS node and MoveIt
        moveit_commander.roscpp_initialize(['/home/mleiter/catkin_ws/src/hw_pkg/leite032_kortex_angle.py', '__ns:=my_gen3'])
        rospy.init_node('set_angle_move_it_trajectories')

        try:
            # Parameters related to gripper and robot DOF
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""

            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # MoveIt interfaces
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def reach_joint_angles(self, tolerance, angle_shoulder, angle_elbow):
        """
        Commands the robotic arm to move to the specified joint angles for shoulder and elbow.
        """
        arm_group = self.arm_group
        success = True

        # Get current joint values
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Current joint positions:")
        for p in joint_positions:
            rospy.loginfo(p)

        # Set movement tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Map estimated angles to Kinova joints depending on DOF
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0
            joint_positions[1] = angle_shoulder
            joint_positions[2] = 0
            joint_positions[3] = 0
            joint_positions[4] = 0
            joint_positions[5] = angle_elbow
            joint_positions[6] = 0
        elif self.degrees_of_freedom == 6:
            # Adapt elbow and shoulder logic for 6 DOF robot
            joint_positions[0] = 0
            joint_positions[1] = angle_shoulder
            joint_positions[2] = 0
            joint_positions[3] = angle_elbow
            joint_positions[4] = 0
            joint_positions[5] = 0

        arm_group.set_joint_value_target(joint_positions)
        success &= arm_group.go(wait=True)

        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("New joint positions after movement:")
        for p in new_joint_positions:
            rospy.loginfo(p)
        return success

# Instantiate the class to control the robot
set_angle = SetAngleMoveItTrajectories()

# MediaPipe setup for pose tracking
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Initialize video capture from first available camera
for camera_idx in range(10):
    vid = cv2.VideoCapture(camera_idx)
    vid.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    if vid.isOpened():
        break

# Infinite loop to process each frame
while(True):
    ret, frame = vid.read()
    image = frame

    with mp_pose.Pose(static_image_mode=True, min_detection_confidence=0.5, model_complexity=2) as pose:
        results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    if results.pose_landmarks is not None:
        # Get joint positions from MediaPipe
        left_shoulder = results.pose_landmarks.landmark[mp.solutions.holistic.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = results.pose_landmarks.landmark[mp.solutions.holistic.PoseLandmark.RIGHT_SHOULDER]
        right_elbow = results.pose_landmarks.landmark[mp.solutions.holistic.PoseLandmark.RIGHT_ELBOW]
        right_wrist = results.pose_landmarks.landmark[mp.solutions.holistic.PoseLandmark.RIGHT_WRIST]

        # Check visibility of landmarks before acting
        if right_shoulder.visibility > 0.8 and right_elbow.visibility > 0.8 and right_wrist.visibility > 0.8:
            collarbone = np.array((left_shoulder.x - right_shoulder.x, left_shoulder.y - right_shoulder.y))
            upper_arm = np.array((right_shoulder.x - right_elbow.x, right_shoulder.y - right_elbow.y))
            lower_arm = np.array((right_elbow.x - right_wrist.x, right_elbow.y - right_wrist.y))

            # Compute joint angles from vectors using cosine rule
            cos_angle_elbow = np.dot(upper_arm, lower_arm) / (np.linalg.norm(upper_arm) * np.linalg.norm(lower_arm))
            cos_angle_shoulder = np.dot(collarbone, upper_arm) / (np.linalg.norm(collarbone) * np.linalg.norm(upper_arm))

            # Clamp and convert to joint angle values
            angle_elbow = min(max(np.arccos(cos_angle_elbow) - pi / 2, -pi/2), pi/2)
            angle_shoulder = min(max(np.arccos(cos_angle_shoulder), 0), pi/2)

            # Send joint angles to robot
            set_angle.reach_joint_angles(0.01, angle_shoulder, angle_elbow)
            print(angle_elbow)

    # Draw pose landmarks on frame
    mp_drawing.draw_landmarks(
        image,
        results.pose_landmarks,
        mp_pose.POSE_CONNECTIONS,
        landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

    cv2.imshow('image', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
vid.release()
cv2.destroyAllWindows()
moveit_commander.roscpp_shutdown()
