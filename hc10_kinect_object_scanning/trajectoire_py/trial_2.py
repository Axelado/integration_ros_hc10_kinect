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
actual_pose, output, center_point, i, init = [], [], [], 0, False

def process_initial_points(points, trans, center_point_, actual_pose, scale, eps):
    global desired_point_stamped, center_point, output

    desired_point_stamped.point.x = center_point_[0]
    desired_point_stamped.point.y = center_point_[1]
    desired_point_stamped.point.z = 0
    center_point = tf2_geometry_msgs.do_transform_point(desired_point_stamped, trans)

    min_Z, max_Z = np.min(points[:, 2]), np.max(points[:, 2])

    ext_x = points[points[:, 2] == max_Z][np.argmax(points[points[:, 2] == max_Z][:, 0])]
    ext_y = points[points[:, 2] == max_Z][np.argmax(points[points[:, 2] == max_Z][:, 1])]
    ext_x = transform_point(ext_x, trans)
    ext_y = transform_point(ext_y, trans)
    dist_x = np.linalg.norm([ext_x.point.x - center_point.point.x, ext_x.point.y - center_point.point.y])
    dist_y = np.linalg.norm([ext_y.point.x - center_point.point.x, ext_y.point.y - center_point.point.y])

    rospy.loginfo(f"x = {dist_x}, y = {dist_y}")

    floor_z_threshold = 0.3 * abs(min_Z - max_Z)
    a, b = dist_x + eps, dist_y + eps
    t = np.linspace(0, 2 * np.pi, 14)
    x = center_point.point.x + a * np.cos(t) * scale
    y = center_point.point.y + b * np.sin(t) * scale
    z1 = (actual_pose[2] - min_Z - floor_z_threshold - 0.01) * np.ones(len(x))

    output = list(np.column_stack((x, y, z1)))


def transform_point(point, trans):
    """Transforme un point en utilisant la transformation donnée."""
    desired_point_stamped.point.x = point[0]
    desired_point_stamped.point.y = point[1]
    desired_point_stamped.point.z = 0
    return tf2_geometry_msgs.do_transform_point(desired_point_stamped, trans)


def execute_movement(points, trans, actual_pose):
    global move_group, output, desired_point_stamped

    desired_point_stamped.point.x = np.mean(points[:, 0])
    desired_point_stamped.point.y = np.mean(points[:, 1])
    desired_point_stamped.point.z = actual_pose[2]
    center_point = tf2_geometry_msgs.do_transform_point(desired_point_stamped, trans)

    for k, desired_point in enumerate(output):
        quaternion = calculate_orientation_towards(center_point, desired_point, trans)

        target_pose.position.x, target_pose.position.y, target_pose.position.z = desired_point
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = quaternion

        move_group.set_pose_target(target_pose)
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        rospy.loginfo(f"Moved to point: n°{k} {target_pose}")

    rospy.sleep(2)


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
    global group_name, move_group, target_pose, desired_point_stamped, output, center_point, i, init, actual_pose

    if rospy.is_shutdown():
        return

    scale, eps = 1.39, 0.1189
    points = []

    try:
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)])
        
        if points.size == 0:
            rospy.logwarn("No points available in the point cloud.")
            return

        from_frame, to_frame = 'kinect_link_optical', 'world'
        trans = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0))

        center = np.mean(points, axis=0)
        distances = np.linalg.norm(points - center, axis=1)
        center_point_ = points[np.argmin(distances)]

        if not init:
            min_Z = np.min(points[:, 2])
            if min_Z >= 0.45:
                actual_pose = np.array([move_group.get_current_pose().pose.position.x,
                                        move_group.get_current_pose().pose.position.y,
                                        move_group.get_current_pose().pose.position.z])
                    
                target_pose.position.x = actual_pose[0]
                target_pose.position.y = actual_pose[1]
                target_pose.position.z = actual_pose[2] - 0.17
                    
                target_pose.orientation.x = 0
                target_pose.orientation.y = 0
                target_pose.orientation.z = 0
                target_pose.orientation.w = 1
                    
                move_group.set_pose_target(target_pose)
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()
                
                init = True
                return

        if i == 0:
            process_initial_points(points, trans, center_point_, actual_pose, scale, eps)
            i += 1
            return

        elif i == 1:
            execute_movement(points, trans, actual_pose)
            
            move_group.set_max_acceleration_scaling_factor(1)
            move_group.set_max_velocity_scaling_factor(1)

            joint_goal = move_group.get_current_joint_values()
            joint_goal[:6] = [-0.328, 0.318, 2.190, 0.013, -1.910, 0.398]

            move_group.set_joint_value_target(joint_goal)
            move_group.go(wait=True)
            move_group.stop()
            
            current_pose = move_group.get_current_pose().pose
            target_pose.position = current_pose.position
            target_pose.orientation = (0, 0, 0, 1)

            move_group.set_pose_target(target_pose)
            move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

            rospy.loginfo("End of movement")
            rospy.signal_shutdown("Shutting down")
            return

    except Exception as e:
        rospy.logerr(f"Failed to process point cloud: {str(e)}")

if __name__ == "__main__":
    rospy.init_node("point_cloud_retrieval", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber("/camera/depth/points_black", PointCloud2, pointcloud_callback, tf_buffer)
    rospy.spin()
