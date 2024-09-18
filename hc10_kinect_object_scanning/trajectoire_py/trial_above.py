#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PointStamped
import moveit_commander
import tf2_ros
import tf.transformations as tfs
import tf2_geometry_msgs

safety_distance = 0.20
update_rate = 10  
i = 0

group_name = "hc10_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
target_pose = Pose()
desired_point_stamped = PointStamped()

def pointcloud_callback(msg, tf_buffer):
    
    global safety_distance, group_name, move_group, target_pose, i, desired_point_stamped
    if not rospy.is_shutdown():  
        points = []
        scale = 1.5
        
        try:
            
            for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
                x, y, z = p[0], p[1], p[2]
                points.append([x, y, z])
            
            points = np.array(points)
            
            if points.size == 0:
                rospy.logwarn("No points available in the point cloud.")
                return
            
            if i == 0:
                
                from_frame = 'kinect_link_optical' 
                to_frame = 'world'
                trans = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0))
                
                min_Z = min(points[:, 2])
                max_Z = max(points[:, 2])
                MinX = min(points[:, 0])
                MinY = min(points[:, 1])
                MaxX = max(points[:, 0])
                MaxY = max(points[:, 1])
                
                top_plan = points[points[:, 2] == min_Z]
                                
                center_point_ = np.mean(top_plan, axis=0)
                
                actual_pose = np.array([move_group.get_current_pose().pose.position.x,
                                        move_group.get_current_pose().pose.position.y,
                                        move_group.get_current_pose().pose.position.z])
                
                desired_point_stamped = PointStamped()
                desired_point_stamped.point.x = center_point_[0]
                desired_point_stamped.point.y = center_point_[1]
                desired_point_stamped.point.z = actual_pose[2]  
                center_point = tf2_geometry_msgs.do_transform_point(desired_point_stamped, trans)
                
                a = 0.2
                b = 0.25
                t = np.linspace(0, 2 * np.pi, 15)
                x = center_point.point.x + a * np.cos(t) * scale
                y = center_point.point.y + b * np.sin(t) * scale
                
                z_start = actual_pose[2]
                z_target = min_Z + 0.05  
                z_steps = np.linspace(z_start, z_target, len(x))

                output = np.column_stack((x, y, z_steps))
                
                def calculate_orientation_towards(center_point, robot_position):
                    direction_vector = np.array([center_point.point.x - robot_position[0],
                                                center_point.point.y - robot_position[1],
                                                max_Z - robot_position[2]])

                    
                    direction_vector_normalized = direction_vector / np.linalg.norm(direction_vector)
                    
                    z_axis = np.array([0, 0, -1]) 
                    x_axis = np.array([1, 0, 0])
                    y_axis = np.cross(z_axis, x_axis)
                    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
                    quaternion = tfs.quaternion_from_matrix(rotation_matrix)
                    
                    return quaternion

                j = 0
                for k, desired_point in enumerate(output):
                    quaternion = calculate_orientation_towards(center_point, desired_point)
                    
                    rospy.loginfo(quaternion)
                    
                    target_pose = Pose()
                    target_pose.position.x = desired_point[0]
                    target_pose.position.y = desired_point[1]
                    target_pose.position.z = desired_point[2]
                    
                    target_pose.orientation.x = quaternion[0]
                    target_pose.orientation.y = quaternion[1]
                    target_pose.orientation.z = quaternion[2]
                    target_pose.orientation.w = quaternion[3]
                    
                    move_group.set_pose_target(target_pose)
                    plan = move_group.go(wait=True)
                    move_group.stop()
                    move_group.clear_pose_targets()

                    rospy.loginfo(f"Moved to point: n°{k} {target_pose}")
                
                rospy.loginfo("End of movement")
                
                move_group.set_max_acceleration_scaling_factor(1)
                move_group.set_max_velocity_scaling_factor(1)

                joint_actuel = move_group.get_active_joints()

                joint_goal = move_group.get_current_joint_values()
                joint_goal[0] = -0.389
                joint_goal[1] = 0.304
                joint_goal[2] = 1.401
                joint_goal[3] = 0.115
                joint_goal[4] = -0.962
                joint_goal[5] = 0.0

                move_group.set_joint_value_target(joint_goal)
                plan = move_group.go(wait=True)
                move_group.stop()
                i += 1
            else:
                rospy.signal_shutdown("Shutting down node.")
        except Exception as e:
            rospy.logerr("Failed to process point cloud: %s" % str(e))


if __name__ == "__main__":
    rospy.init_node("point_cloud_retrieval", anonymous=True)
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber("/camera/depth/points_black", PointCloud2, pointcloud_callback, tf_buffer)
    rospy.spin()