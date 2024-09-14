#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import open3d as o3d
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices

def ros_to_open3d_point_cloud(ros_cloud):
    points = np.array(list(pc2.read_points(ros_cloud, skip_nans=True, field_names=("x", "y", "z"))), dtype=np.float32)
    open3d_cloud = o3d.geometry.PointCloud()
    open3d_cloud.points = o3d.utility.Vector3dVector(points)
    return open3d_cloud

def open3d_to_ros_point_cloud(open3d_cloud, frame_id):
    points = np.asarray(open3d_cloud.points)
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return pc2.create_cloud_xyz32(header, points)

def filter_floor_from_point_cloud(open3d_cloud):
    plane_model, inliers = open3d_cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=100)
    return open3d_cloud.select_by_index(inliers, invert=True)

def transform_point_cloud(open3d_cloud, transformation_matrix):
    # Apply the transformation to each point
    points = np.asarray(open3d_cloud.points)
    homogenous_points = np.hstack((points, np.ones((points.shape[0], 1))))  # Convert to homogeneous coordinates
    transformed_points = homogenous_points @ transformation_matrix.T  # Apply transformation
    transformed_cloud = o3d.geometry.PointCloud()
    transformed_cloud.points = o3d.utility.Vector3dVector(transformed_points[:, :3])  # Convert back to 3D coordinates
    return transformed_cloud

def pointcloud_callback(msg):
    try:
        # Define the transformation matrix (example: translation of 1m along x, 2m along y, 3m along z)
        translation = np.array([1.0, 2.0, 3.0])
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, 3] = translation
        
        open3d_cloud = ros_to_open3d_point_cloud(msg)
        filtered_cloud = filter_floor_from_point_cloud(open3d_cloud)
        
        # Apply transformation to the point cloud
        transformed_cloud = transform_point_cloud(filtered_cloud, transformation_matrix)
        
        filtered_ros_cloud = open3d_to_ros_point_cloud(transformed_cloud, msg.header.frame_id)
        point_cloud_pub.publish(filtered_ros_cloud)
    except Exception as e:
        rospy.logerr(f"Failed to process point cloud: {e}")

if __name__ == "__main__":
    rospy.init_node("kinect_pointcloud_modifier", anonymous=True)
    point_cloud_pub = rospy.Publisher("/camera/depth/points_black", PointCloud2, queue_size=10)
    rospy.Subscriber("/camera/depth/points", PointCloud2, pointcloud_callback)
    rospy.spin()
