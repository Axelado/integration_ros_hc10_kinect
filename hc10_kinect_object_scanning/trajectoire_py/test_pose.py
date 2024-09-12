#!/usr/bin/env python3

import moveit_commander
from geometry_msgs.msg import Pose
groupe_name = "hc10_arm"


move_groupe = moveit_commander.MoveGroupCommander(groupe_name)

values = move_groupe.get_current_joint_values()
print(values)


move_groupe.set_max_acceleration_scaling_factor(1)
move_groupe.set_max_velocity_scaling_factor(1)
print(move_groupe.has_end_effector_link())


plan = move_groupe.go(wait=True)
move_groupe.stop()

target_Pose = Pose()#creation de la target pose dans le bon format

target_Pose.position.x=0.0
target_Pose.position.y=0.0
target_Pose.position.z=0.3

target_Pose.orientation.x=0.0#ox
target_Pose.orientation.y=0.0#oy
target_Pose.orientation.z=0.0#oz
target_Pose.orientation.w=0.0#ow

move_groupe.set_pose_target(target_Pose) # donne a moveit la position goal

plan = move_groupe.go(wait=True) #le bras bouge vers la position de destination

move_groupe.stop() # arrete le mouvement
move_groupe.clear_pose_targets() # supprime la position goal sur moveit

values = move_groupe.get_current_joint_values()
print(values)