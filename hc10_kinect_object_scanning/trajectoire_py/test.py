#!/usr/bin/env python3

import moveit_commander
from geometry_msgs.msg import Pose
groupe_name = "hc10_arm"

import time

move_groupe = moveit_commander.MoveGroupCommander(groupe_name)
robot = moveit_commander.RobotCommander()

values = move_groupe.get_current_joint_values()
print(values)

move_groupe.set_random_target()
move_groupe.set_max_acceleration_scaling_factor(1)
move_groupe.set_max_velocity_scaling_factor(1)
print(move_groupe.has_end_effector_link())
values = move_groupe.get_current_joint_values()
print(values)

plan = move_groupe.go(wait=True)
move_groupe.stop()
robot_state = robot.get_current_state()
print("kjdfi",robot.get_current_state())
print(type(robot_state))
joint_6_t = robot_state.get_global_link_transform("joint_6_t")
print("edfiuhspogjs",joint_6_t)
time.sleep(3)
joint_actuel = move_groupe.get_active_joints()

values = move_groupe.get_current_joint_values()
print(values)

joint_goal = move_groupe.get_current_joint_values()
joint_goal[0] = -0.47
joint_goal[1] = -0.08
joint_goal[2] = 0.82
joint_goal[3] = 0.09
joint_goal[4] = -0.80
joint_goal[5] = 0.0


move_groupe.set_joint_value_target(joint_goal)

plan = move_groupe.go(wait=True)

move_groupe.stop()

values = move_groupe.get_current_joint_values()
print(values)
