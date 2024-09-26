#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import traceback

i, eps = 0, 0.1

def pointcloud_callback(msg):
    global i, eps

    try:
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])

        if points.shape[0] == 0:
            rospy.logwarn("No points in point cloud.")
            return
        
        def fit_plane(points):
            centroid = np.mean(points, axis=0)
            centered_points = points - centroid
            cov_matrix = np.cov(centered_points.T)
            eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
            normal = eigenvectors[:, np.argmin(eigenvalues)]
            d = -normal.dot(centroid)
            return normal, d

        def point_to_plane_distance(points, normal, d):
            return np.abs(np.dot(points, normal) + d) / np.linalg.norm(normal)

        def remove_plane_points(points, threshold=0.01):
            normal, d = fit_plane(points)
            distances = point_to_plane_distance(points, normal, d)
            return points[distances > threshold]

        output = remove_plane_points(points)

        if output.shape[0] == 0:
            rospy.logwarn("No points remaining after filtering.")
            return

        min_z, max_z = np.min(output[:, 2]), np.max(output[:, 2])
        floor_z_threshold = 0.196 * (abs(max_z - min_z))
        output = output[(output[:, 2] >= min_z) & (output[:, 2] <= max_z - floor_z_threshold)]

        rospy.loginfo(f"Filtered points: max z = {np.max(output[:, 2])}, min z = {np.min(output[:, 2])}, threshold = {floor_z_threshold}, eps = {eps}")

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id
        filtered_ros_cloud = pc2.create_cloud_xyz32(header, output)
        point_cloud_pub.publish(filtered_ros_cloud)

    except Exception as e:
        rospy.logerr(f"Failed to process point cloud: {str(e)}")
        rospy.logerr(traceback.format_exc())

if __name__ == "__main__":
    rospy.init_node("kinect_pointcloud_modifier", anonymous=True)
    
    point_cloud_pub = rospy.Publisher("/camera/depth/points_black", PointCloud2, queue_size=10)
    rospy.Subscriber("/camera/depth/points", PointCloud2, pointcloud_callback)
    
    rospy.spin()
