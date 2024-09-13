#!/usr/bin/env python3

import moveit_commander
from geometry_msgs.msg import Pose
groupe_name = "hc10_arm"


move_groupe = moveit_commander.MoveGroupCommander(groupe_name)

target_Pose = Pose()

target_Pose.position.x=0.2
target_Pose.position.y=0.6
target_Pose.position.z=0.8

target_Pose.position.x=-0.45
target_Pose.position.y=0.9
target_Pose.position.z=0.3

target_Pose.orientation.x=0.0#ox
target_Pose.orientation.y=0.0#oy
target_Pose.orientation.z=0.0#oz
target_Pose.orientation.w=0.0#ow

move_groupe.set_pose_target(target_Pose, end_effector_link='kinect_link') # donne a moveit la position goal

plan = move_groupe.go(wait=False) #le bras bouge vers la position de destination