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

group_name = "hc10_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
target_pose = Pose()
desired_point_stamped = PointStamped()
actual_pose, final_pose, output, center_point, i = [], [], [], [], 0


def calculate_orientation_towards(center_point, robot_position, trans):
    direction_vector = np.array([center_point.point.x - robot_position[0],
                                 center_point.point.y - robot_position[1],
                                 center_point.point.z - robot_position[2]])

    direction_vector_normalized = direction_vector / np.linalg.norm(direction_vector)

    quaternion = trans.transform.rotation
    rotation_matrix = tfs.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    z_axis = rotation_matrix[:3, 2]

    rotation_matrix = tfs.rotation_matrix(np.arccos(np.dot(z_axis, direction_vector_normalized)),
                                          np.cross(z_axis, direction_vector_normalized))
    quaternion = tfs.quaternion_from_matrix(rotation_matrix)
    return quaternion


def pointcloud_callback(msg, tf_buffer):
    
    global group_name, move_group, target_pose, desired_point_stamped, output, center_point, i, actual_pose, final_pose
    if not rospy.is_shutdown():
        
        scale = 1.38
        eps = 0.12
        points = []

        try:
            for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
                x, y, z = p[0], p[1], p[2]
                points.append([x, y, z])

            points = np.array(points)

            from_frame = 'kinect_link_optical'
            to_frame = 'world'
            trans = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0))

            if points.size == 0:
                rospy.logwarn("No points available in the point cloud.")
                return  
            
            center_point_ = np.mean(points, axis=0)
            
            if i == 0:
                
                center_point_ = np.mean(points, axis=0)
                min_Z = np.min(points[:, 2])
                max_Z = np.max(points[:, 2])
                
                actual_pose = np.array([move_group.get_current_pose().pose.position.x,
                                        move_group.get_current_pose().pose.position.y,
                                        move_group.get_current_pose().pose.position.z])
                
                desired_point_stamped.point.x = center_point_[0]
                desired_point_stamped.point.y = center_point_[1]
                desired_point_stamped.point.z = min_Z
                center_point = tf2_geometry_msgs.do_transform_point(desired_point_stamped, trans)
                
                
                all_max_z = points[points[:, 2] == max_Z]
                ext_x = all_max_z[np.argmax(all_max_z[:, 0])]
                ext_y = all_max_z[np.argmax(all_max_z[:, 1])]
                
                desired_point_stamped.point.x = ext_x[0]
                desired_point_stamped.point.y = ext_x[1]
                desired_point_stamped.point.z = max_Z
                ext_x = tf2_geometry_msgs.do_transform_point(desired_point_stamped, trans)
                
                desired_point_stamped.point.x = ext_y[0]
                desired_point_stamped.point.y = ext_y[1]
                desired_point_stamped.point.z = min_Z
                ext_y = tf2_geometry_msgs.do_transform_point(desired_point_stamped, trans)
                
                dist_x = np.linalg.norm([ext_x.point.x - center_point.point.x, ext_x.point.y - center_point.point.y])
                dist_y = np.linalg.norm([ext_y.point.y - center_point.point.y, ext_y.point.y - center_point.point.y])
                
                rospy.loginfo(f"x = {dist_y}, y = {dist_y}")

                actual_pose = np.array([move_group.get_current_pose().pose.position.x,
                                        move_group.get_current_pose().pose.position.y,
                                        move_group.get_current_pose().pose.position.z])

                
                floor_z_threshold = 0.13 * abs(ext_x.point.z - ext_y.point.z)

                a = dist_x + eps
                b = dist_y + eps

                t = np.linspace(0, 2 * np.pi, 14)
                x = center_point.point.x + a * np.cos(t) * scale
                y = center_point.point.y + b * np.sin(t) * scale
                z1 = (actual_pose[2] - min_Z - floor_z_threshold + 0.002) * np.ones(len(x))
                
                output = list(np.column_stack((x, y, z1)))
                
                current_pose = move_group.get_current_pose().pose

                target_pose = Pose()
                target_pose.position.x = center_point.point.x
                target_pose.position.y = center_point.point.y
                target_pose.position.z = current_pose.position.z

                target_pose.orientation.x = 0
                target_pose.orientation.y = 0
                target_pose.orientation.z = 0
                target_pose.orientation.w = 1  

                move_group.set_pose_target(target_pose)
                move_group.go(wait=True)

                move_group.stop()
                move_group.clear_pose_targets()

                final_pose = move_group.get_current_pose().pose

                i += 1
                return

            elif i == 1:
                
                center_point_ = np.mean(points, axis=0)
                
                desired_point_stamped.point.x = center_point_[0]
                desired_point_stamped.point.y = center_point_[1]
                desired_point_stamped.point.z = actual_pose[2]
                center_point = tf2_geometry_msgs.do_transform_point(desired_point_stamped, trans)

                for k, desired_point in enumerate(output):
                    quaternion = calculate_orientation_towards(center_point, desired_point, trans)

                    target_pose.position.x = desired_point[0]
                    target_pose.position.y = desired_point[1]
                    target_pose.position.z = desired_point[2]

                    target_pose.orientation.x = quaternion[0]
                    target_pose.orientation.y = quaternion[1]
                    target_pose.orientation.z = quaternion[2]
                    target_pose.orientation.w = quaternion[3]

                    move_group.set_pose_target(target_pose)
                    move_group.go(wait=True)
                    move_group.stop()
                    move_group.clear_pose_targets()

                    rospy.loginfo(f"Moved to point: n°{k} {target_pose}")

                rospy.sleep(2)

                
                current_pose = final_pose

                target_pose = Pose()
                target_pose.position.x = current_pose.position.x
                target_pose.position.y = current_pose.position.y
                target_pose.position.z = current_pose.position.z

                target_pose.orientation.x = 0
                target_pose.orientation.y = 0
                target_pose.orientation.z = 0
                target_pose.orientation.w = 1  

                move_group.set_pose_target(target_pose)
                move_group.go(wait=True)

                move_group.stop()
                move_group.clear_pose_targets()

                final_joint_values = move_group.get_current_joint_values()

                rospy.loginfo("End of movement")
                rospy.signal_shutdown("Shutting down")
                return

        except Exception as e:
            rospy.logerr("Failed to process point cloud: %s" % str(e))


if __name__ == "__main__":
    rospy.init_node("point_cloud_retrieval", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber("/camera/depth/points_black", PointCloud2, pointcloud_callback, tf_buffer)
    rospy.spin()
