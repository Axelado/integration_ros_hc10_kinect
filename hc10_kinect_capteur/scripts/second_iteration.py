#!/usr/bin/env python3

import rospy
import numpy as np
import random
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
from tf import TransformListener
import traceback


def pointcloud_callback(msg):
    try:
        distance_threshold = 0.1
        target_frame = "/world"
        source_frame = msg.header.frame_id

        points = []
        
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            points.append([x, y, z])
        
        if len(points) == 0:
            rospy.logwarn("No points in point cloud.")
            return

        points = np.array(points)

        best_inliers = []
        best_plane_coeffs = None
        distance_threshold = 0.1
        ransac_n = 3
        num_iterations = 100
        

        for _ in range(num_iterations):
            sample_indices = random.sample(range(points.shape[0]), ransac_n)
            sample_points = points[sample_indices]

            if np.linalg.matrix_rank(sample_points - sample_points[0]) < 3:
                continue

            vec1 = sample_points[1] - sample_points[0]
            vec2 = sample_points[2] - sample_points[0]

            normal = np.cross(vec1, vec2)
            normal /= np.linalg.norm(normal)

            a, b, c = normal
            d = -np.dot(normal, sample_points[0])

            distances = np.abs(np.dot(points, normal) + d) / np.linalg.norm(normal)

            inliers = np.where(distances < distance_threshold)[0]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_plane_coeffs = (a, b, c, d)

        output = points[~np.isin(np.arange(points.shape[0]), best_inliers)]

        min_z, max_z = min(output[:, 2]), max(output[:, 2])
        floor_z_threshold = 0.8*(abs(min_z -max_z))

        output = output[(output[:, 2] >= min_z) & (output[:, 2] <= (max_z - floor_z_threshold))]
        
        rospy.loginfo(f"2 - max = {max(output[:, 2])}, min = {min(output[:, 2])}, th = {floor_z_threshold}")
        
        if output.shape[0] == 0:
            rospy.logwarn("No points remaining after filtering.")
            return
        
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
