#!/usr/bin/env python3

import moveit_commander
from geometry_msgs.msg import Pose

groupe_name = "hc10_arm"
move_groupe = moveit_commander.MoveGroupCommander(groupe_name)
robot = moveit_commander.RobotCommander()

move_groupe.set_max_acceleration_scaling_factor(1)
move_groupe.set_max_velocity_scaling_factor(1)

joint_actuel = move_groupe.get_active_joints()

joint_goal = move_groupe.get_current_joint_values()
joint_goal[0] = -0.328
joint_goal[1] = 0.318
joint_goal[2] = 2.190
joint_goal[3] = 0.013
joint_goal[4] = -1.910
joint_goal[5] = 0.398

move_groupe.set_joint_value_target(joint_goal)

plan = move_groupe.go(wait=True)
move_groupe.stop()

current_pose = move_groupe.get_current_pose().pose

target_pose = Pose()
target_pose.position.x = current_pose.position.x
target_pose.position.y = current_pose.position.y
target_pose.position.z = current_pose.position.z

target_pose.orientation.x = 0
target_pose.orientation.y = 0
target_pose.orientation.z = 0
target_pose.orientation.w = 1  

move_groupe.set_pose_target(target_pose)
move_groupe.go(wait=True)

move_groupe.stop()
move_groupe.clear_pose_targets()

final_joint_values = move_groupe.get_current_joint_values()
print(final_joint_values)
