#!/usr/bin/env python3

import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose
groupe_name = "hc10_arm"

move_groupe = moveit_commander.MoveGroupCommander(groupe_name)

#joint_actuel = move_groupe.get_active_joints()
#
#values = move_groupe.get_current_joint_values()
#print(values)
#
#
#joint_goal = move_groupe.get_current_joint_values()
#joint_goal[0] = 0.0
#joint_goal[1] = 0.0
#joint_goal[2] = 0.0
#joint_goal[3] = 0.0
#joint_goal[4] = 0.0
#joint_goal[5] = 0.0
#
#
#move_groupe.set_joint_value_target(joint_goal)
#
#plan = move_groupe.go(wait=True)
#
#move_groupe.stop()
#
#values = move_groupe.get_current_joint_values()
#print(values)

values = move_groupe.get_current_pose()
print(values)

target_Pose = Pose()
target_Pose.position.x=0.0
target_Pose.position.y=0.0
target_Pose.position.z=0.3

target_Pose.orientation.x=0.0
target_Pose.orientation.y=0.0
target_Pose.orientation.z=0.0
target_Pose.orientation.w=0.0

#move_groupe.set_pose_target(target_Pose)

joint_goal = move_groupe.get_joint_value_target()
print("joint goal :", joint_goal)

move_groupe.set_joint_value_target(joint_goal)

#
#plan = move_groupe.go(wait=True)
#
#move_groupe.stop()
#move_groupe.clear_pose_targets()
#
#moveit_commander.roscpp_shutdown()