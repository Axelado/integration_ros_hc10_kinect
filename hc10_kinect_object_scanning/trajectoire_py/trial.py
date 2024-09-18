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
output, center_point, h = [], [], False

def calculate_orientation_bellow(center_point, robot_position):
    
    direction_vector = np.array([center_point.point.x - robot_position[0],
    center_point.point.y - robot_position[1],
    center_point.point.z - robot_position[2]])

    z_axis = np.array([0, 0, 1])
    x_axis = np.array([1, 0, 0])
    y_axis = np.cross(z_axis, x_axis)
    
    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
    quaternion = tfs.quaternion_from_matrix(rotation_matrix)

    return quaternion


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

    global safety_distance, group_name, move_group, target_pose, i, desired_point_stamped, output, center_point, h
    if not rospy.is_shutdown():  
        points = []
        scale = 1.2
                
        try:    
            for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
                x, y, z = p[0], p[1], p[2]
                points.append([x, y, z])
            
            points = np.array(points)
            
            from_frame = 'kinect_link_optical' 
            to_frame = 'world'
            trans = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0))
                
            min_Z = min(points[:, 2])

            top_plan = points[points[:, 2] == min_Z]
                
            center_point_ = np.mean(top_plan, axis=0)
            
            if points.size == 0:
                rospy.logwarn("No points available in the point cloud.")
                return
            
            if i == 0:    
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
                z = (actual_pose[2] - min_Z + 0.03) * np.ones(len(x))
        
                output = list(np.column_stack((x, y, z)))
                output.append(actual_pose)
                
                i += 1
                
            elif i != 0 and (not h):
                        
                actual_pose = np.array([move_group.get_current_pose().pose.position.x,
                                            move_group.get_current_pose().pose.position.y,
                                            move_group.get_current_pose().pose.position.z])
                    
                desired_point_stamped = PointStamped()
                desired_point_stamped.point.x = center_point_[0]
                desired_point_stamped.point.y = center_point_[1]
                desired_point_stamped.point.z = actual_pose[2]  
                center_point = tf2_geometry_msgs.do_transform_point(desired_point_stamped, trans)
                    
                for k, desired_point in enumerate(output):
            
                    quaternion = calculate_orientation_towards(center_point, desired_point, trans)
                    
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
                
                h = True
                output = output[::-2]
                
                rospy.loginfo("end of movement")
                rospy.signal_shutdown("Shuting down")
                
        except Exception as e:
            rospy.logerr("Failed to process point cloud: %s" % str(e))

if __name__ == "__main__":
    rospy.init_node("point_cloud_retrieval", anonymous=True)
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber("/camera/depth/points_black", PointCloud2, pointcloud_callback, tf_buffer)
    rospy.spin()
