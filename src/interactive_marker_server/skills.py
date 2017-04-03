import rospy
import copy
from math import sin, cos

import graspit_msgs.msg
import graspit_commander
import actionlib
import geometry_msgs
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import (MoveGroupInterface, PickPlaceInterface)

#Move the base
def goto(client, x, y, frame="map"):
    move_goal = MoveBaseGoal()
    move_goal.target_pose.pose.position.x = x
    move_goal.target_pose.pose.position.y = y
    # move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
    # move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
    move_goal.target_pose.pose.orientation.z = 0.998
    move_goal.target_pose.pose.orientation.w = -0.055
    move_goal.target_pose.header.frame_id = frame
    move_goal.target_pose.header.stamp = rospy.Time.now()
    # TODO wait for things to work
    client.send_goal(move_goal)
    client.wait_for_result()

#Follow the trajectory
def move_to(client, joint_names, positions, duration=5.0):
    if len(joint_names) != len(positions):
        print("Invalid trajectory position")
        return False
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = positions
    trajectory.points[0].velocities = [0.0 for _ in positions]
    trajectory.points[0].accelerations = [0.0 for _ in positions]
    trajectory.points[0].time_from_start = rospy.Duration(duration)
    follow_goal = FollowJointTrajectoryGoal()
    follow_goal.trajectory = trajectory

    client.send_goal(follow_goal)
    client.wait_for_result()

#Point head on detected object
def look_at(client, x, y, z, frame, duration=1.0):
    goal = PointHeadGoal()
    goal.target.header.stamp = rospy.Time.now()
    goal.target.header.frame_id = frame
    goal.target.point.x = x
    goal.target.point.y = y
    goal.target.point.z = z
    goal.min_duration = rospy.Duration(duration)
    client.send_goal(goal)
    client.wait_for_result()

   #Tuck the arm
    
def tuck():
    move_group = MoveGroupInterface("arm", "base_link")
    joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    while not rospy.is_shutdown():
        result = move_group.moveToJointPosition(joints, pose, 0.02)

def place(reachable_grasps):
    rospy.loginfo("About to place the object ...")
    place_client = actionlib.SimpleActionClient('place_execution_action', graspit_msgs.msg.PlaceExecutionAction)
    place_client.wait_for_server()
    goal = graspit_msgs.msg.PlaceExecutionGoal()
    goal.grasp = reachable_grasps[0]
    place_client.send_goal(goal)    
    place_client.wait_for_result()
    


def grasp(model_name):
    rospy.loginfo("Calling the get_grasp call back...")
  
    gc = graspit_commander.GraspitCommander()

    rospy.loginfo("About to plan grasps in Graspit")
    gc.clearWorld()
    gc.importObstacle("table")
    table_pose = geometry_msgs.msg.Pose()
    table_pose.orientation.w = 1
    table_pose.position.x = 0.53
    table_pose.position.y = -0.687
    table_pose.position.z = 0.445
    gc.setBodyPose(0,table_pose)

    gc.importRobot("fetch_gripper")

    gc.importGraspableBody(model_name) #Find the right model using goal, can't find gillete in the folder

    response = gc.planGrasps()
    unchecked_for_reachability_grasps = response.grasps
    rospy.loginfo("We have received grasps from Graspit")

    rospy.loginfo("checking grasps for reachability")
    reachability_client = actionlib.SimpleActionClient('analyze_grasp_action', graspit_msgs.msg.CheckGraspReachabilityAction)
    reachability_client.wait_for_server()

    reachable_grasps = []
    import IPython
    IPython.embed()
    #Grasp needs to be transford by world space coords of object (which is potentially rotated also)
    #Also the rendering needs to take into account the gripper dof and position
    for i, unchecked_grasp in enumerate(unchecked_for_reachability_grasps):
        rospy.loginfo("checking grasps for reachability")

        grasp = graspit_msgs.msg.Grasp()
        grasp.grasp_id = i

        grasp.object_name = model_name
        pre_grasp_pose = geometry_msgs.msg.Pose()
        pre_grasp_pose.position = unchecked_grasp.pose.position
        pre_grasp_pose.orientation = unchecked_grasp.pose.orientation
        grasp.pre_grasp_pose = pre_grasp_pose

        final_grasp_pose = geometry_msgs.msg.Pose()
        final_grasp_pose.position = unchecked_grasp.pose.position
        final_grasp_pose.orientation = unchecked_grasp.pose.orientation
        #print "here mf: \n\n\n\n\n\n\n\n" + str(final_grasp_pose.position.x)
        #final_grasp_pose.position.x = tmp/50.0
        grasp.final_grasp_pose = final_grasp_pose

        grasp.pre_grasp_dof = [0.09] #Maximum cm the gripper can open #copy in from graspit commander
        grasp.final_grasp_dof = unchecked_grasp.dofs #have the gripper close all the way for now

        #this is the message we are sending to reachability analyzer to check for reachability
        goal = graspit_msgs.msg.CheckGraspReachabilityGoal()
        goal.grasp = grasp

        reachability_client.send_goal(goal)    
        reachability_client.wait_for_result()

        reachability_check_result = reachability_client.get_result()

        if reachability_check_result.isPossible:
            reachable_grasps.append(grasp)

    if len(reachable_grasps):
        rospy.loginfo("going to execute first reachable grasp")
        execution_client = actionlib.SimpleActionClient('grasp_execution_action', graspit_msgs.msg.GraspExecutionAction)
        execution_client.wait_for_server()

        goal = graspit_msgs.msg.GraspExecutionGoal()
        goal.grasp = reachable_grasps[0]

        execution_client.send_goal(goal)    
        execution_client.wait_for_result()
    else:
        rospy.loginfo("No reachable grasps found")

    return reachable_grasps

# def _get_grasps_as_cb(self, goal):
#         print("_get_grasps_as_cb")
#         print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
#         _result = graspit_msgs.msg.GetGraspsResult()
#         print("graspit_msgs.msg.GetGraspsResult()")
#         #goal is empty!!!!!!
#         #roslaunch the pr2_cup, close out of stuff I don't need. keep gazebo there. 

#         #for model in self.model_manager.model_list:
#         #    object_info = graspit_msgs.msg.ObjectInfo(model.object_name, model.model_name, model.get_world_pose())

#         #grasps = graspit_commander.plan()

#         #Load XML file to plan grasps on


#         # grasp.epsilon_quality = 0
#         # grasp.volume_quality = 0
#         # grasp.grasp_source = 1
#         # grasp.grasp_group = 1
#         # grasp.grasp_type = 1
#         gc = graspit_commander.GraspitCommander()

#         gc.clearWorld()
#         gc.importRobot("pr2_gripper_2010")
#         gc.importGraspableBody(goal.name) #Find the right model using goal, can't find gillete in the folder
#         #gc.importObstacle("") #Do this later


#         #gc.loadWorld("pr2_ws") #Load the object instead of the world, load obstacle (plane)
#         response = gc.planGrasps()
#         grasps = response.grasps

#         #dof is an angle, need to convert to distance from 0-.09
#         #Note: grasps is in the object's frame of reference, we need to transform by world space

#         #Build up the graspit_mgs grasp using the returned graspit_interface list of grasps

#         #gc.clearWorld -> check if there is an object in scene

#         import IPython
#         IPython.embed()

#         #Grasp needs to be transford by world space coords of object (which is potentially rotated also)
#         #Also the rendering needs to take into account the gripper dof and position
#         for tmp in range(0,len(grasps)):
#             grasp = graspit_msgs.msg.Grasp()
#             grasp.object_name = str(tmp)
#             g = grasps[tmp]
#             grasp.grasp_id = tmp
#             grasp = graspit_msgs.msg.Grasp()
#             grasp.object_name = "garbage"
#             pre_grasp_pose = geometry_msgs.msg.Pose()
#             pre_grasp_pose.position = g.pose.position
#             pre_grasp_pose.orientation = g.pose.orientation
#             grasp.pre_grasp_pose = pre_grasp_pose

#             final_grasp_pose = geometry_msgs.msg.Pose()
#             final_grasp_pose.position = g.pose.position
#             final_grasp_pose.orientation = g.pose.orientation
#             #print "here mf: \n\n\n\n\n\n\n\n" + str(final_grasp_pose.position.x)
#             #final_grasp_pose.position.x = tmp/50.0
#             grasp.final_grasp_pose = final_grasp_pose

#             grasp.pre_grasp_dof = [0.09] #Maximum cm the gripper can open #copy in from graspit commander
#             grasp.final_grasp_dof = g.dofs #have the gripper close all the way for now
#             _result.grasps.append(grasp)

 
#         #_result.grasps.extend(grasps)
#         #_result.grasps = grasps
#         self.get_grasps_as.set_succeeded(_result)
#         return []
