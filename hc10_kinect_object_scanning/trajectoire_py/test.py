#!/usr/bin/env python3

import moveit_commander
groupe_name = "hc10_arm"

move_groupe = moveit_commander.MoveGroupCommander(groupe_name)
robot = moveit_commander.RobotCommander()


move_groupe.set_max_acceleration_scaling_factor(1)
move_groupe.set_max_velocity_scaling_factor(1)

joint_actuel = move_groupe.get_active_joints()


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