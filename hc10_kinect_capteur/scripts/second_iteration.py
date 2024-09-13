#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import open3d as o3d

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
    plane_model, inliers = open3d_cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    return open3d_cloud.select_by_index(inliers, invert=True)

def pointcloud_callback(msg):
    try:
        open3d_cloud = ros_to_open3d_point_cloud(msg)
        filtered_cloud = filter_floor_from_point_cloud(open3d_cloud)
        filtered_ros_cloud = open3d_to_ros_point_cloud(filtered_cloud, msg.header.frame_id)
        point_cloud_pub.publish(filtered_ros_cloud)
    except Exception as e:
        rospy.logerr(f"Failed to process point cloud: {e}")

if __name__ == "__main__":
    rospy.init_node("kinect_pointcloud_modifier", anonymous=True)
    point_cloud_pub = rospy.Publisher("/camera/depth/points_black", PointCloud2, queue_size=10)
    rospy.Subscriber("/camera/depth/points", PointCloud2, pointcloud_callback)
    rospy.spin()