#!/usr/bin/env python
import skills 
from marker_utils import *

import os
import time
import sys
import importlib

import roslib
import rospy
import moveit_commander
import graspit_commander
import tf
import actionlib
import tf.transformations
import sys

import geometry_msgs.msg
from std_srvs.srv import Empty
import moveit_msgs
from moveit_msgs.msg import PlanningSceneComponents
import moveit_msgs.srv
from graspit_msgs.srv import *
import graspit_msgs.msg
import moveit_msgs.msg
import control_msgs.msg


from moveit_commander import PlanningSceneInterface

from reachability_analyzer.grasp_reachability_analyzer import GraspReachabilityAnalyzer

import rospy
import copy
from math import sin, cos

from random import random
from math import sin
import numpy
import math

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from grasp_execution.robot_interface import RobotInterface
from grasp_execution import execution_stages
from grasp_execution import execution_pipeline

from grasp_execution.hand_managers import hand_manager
from grasp_execution.grasp_execution_node import GraspExecutionNode
from reachability_analyzer.grasp_analyzer_node import GraspAnalyzerNode
# from moveit_python import (MoveGroupInterface,
#                            PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes

import trajectory_msgs.msg
import std_msgs.msg
import graspit_msgs.msg


class InteractiveMarkerServerNode():

	def __init__(self):

		rospy.init_node('interactive_marker_server')
		rospy.loginfo("starting interactive marker server...")
		rospy.Subscriber("/run_recognition",  std_msgs.msg.Empty, self.run_recognition)
		self.torso_name = "torso_controller"
		self.torso_joint_names = ["torso_lift_joint"]

		self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		self.torso_client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % self.torso_name,
											   FollowJointTrajectoryAction)
		

		self.head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
		self.rec_objects_action_client = actionlib.SimpleActionClient("recognize_objects_action", graspit_msgs.msg.RunObjectRecognitionAction)
		self.tf_listener = tf.TransformListener()


		self.server = InteractiveMarkerServer("basic_controls")

		self.menu_handler = MenuHandler()

		self.menu_handler.insert( "Go There!", callback=self.go_there_feedback)
		self.menu_handler.insert( "Grasp It!", callback=self.grasp_feedback)
		self.menu_handler.insert( "Up!", callback=self.up_feedback )
		self.menu_handler.insert( "Down!", callback=self.down_feedback )
		self.menu_handler.insert( "Focus on Object!", callback=self.focus_feedback )
		self.menu_handler.insert( "Tuck Arm!", callback=self.tuck_feedback )
		
		# position = Point(0, 0, 0)
		# make6DofMarker(False, "base_link", InteractiveMarkerControl.NONE, position, True)
		self.server.applyChanges()
		rospy.loginfo("interactive marker server starts up successfully...")

	def run_recognition(self, msg):
		rospy.loginfo("entering run recognition call back...")
		goal = graspit_msgs.msg.RunObjectRecognitionActionGoal()
   
		# TODO wait for things to work
		self.rec_objects_action_client.send_goal(goal)
		rospy.loginfo("send run recognition goal...")
		self.rec_objects_action_client.wait_for_result()
		rospy.loginfo("reveiving result...")
		result = self.rec_objects_action_client.get_result()

		for object_info in result.object_info:
			position = object_info.object_pose.position
			position = Point(position.x, position.y, position.z)
			make6DofMarker(fixed=False, 
				frame=object_info.model_name,
				interaction_mode=InteractiveMarkerControl.MOVE_ROTATE_3D,
				position=position, 
				server=self.server,
				menu_handler=self.menu_handler,
				show_6dof = True)
			rospy.loginfo("get model info..." + str(object_info.model_name))

		self.server.applyChanges()



	def go_there_feedback(self, feedback):

		rospy.loginfo("entering go_there_feedback")
		rospy.loginfo("Waiting for move_base server...")
		self.move_base_client.wait_for_server()
			
		(trans, rot) = self.tf_listener.lookupTransform('/map', '/world', rospy.Time(0))
		x = feedback.pose.position.x + trans[0]
		y = feedback.pose.position.y + trans[1]
		skills.goto(self.move_base_client, x, y, 1.57)

		self.server.applyChanges()

	def grasp_feedback(self,feedback):
		model_name = feedback.header.frame_id
		rospy.loginfo("planning grasp for " + str(model_name))
		skills.grasp(model_name)
		rospy.loginfo("entering grasp_feedback")

	# Up
	def up_feedback(self,feedback):
		rospy.loginfo("entering up_feedback")
		rospy.loginfo("menu entry_id " + str(feedback.menu_entry_id))
		rospy.loginfo("Raising torso...")
		self.torso_movement_helper([0.4, ])

	# Down
	def down_feedback(self,feedback):
		rospy.loginfo("entering down_feedback")
		rospy.loginfo("Lowering torso...")
		self.torso_movement_helper([0.1, ])
   

	def focus_feedback(self,feedback):
		rospy.loginfo("entering focus_feedback")
		rospy.loginfo("Waiting for head_controller...")
		self.head_client.wait_for_server()
		rospy.loginfo("Looking at the object...")
		skills.look_at(self.head_client, feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z, "world")

	def tuck_feedback(self,feedback):
		rospy.loginfo("entering tuck_feedback")
		skills.tuck()

	def torso_movement_helper(self, positions):
		rospy.loginfo("Waiting for %s..." % self.torso_name)
		self.torso_client.wait_for_server()
		rospy.loginfo("Lowering torso...")
		skills.move_to(self.torso_client, self.torso_joint_names, positions)


if __name__ == '__main__':

	try:
		print "i am here"
		marker_server = InteractiveMarkerServerNode()


		loop = rospy.Rate(30)
		while not rospy.is_shutdown():
			loop.sleep()

	except rospy.ROSInterruptException:
		pass