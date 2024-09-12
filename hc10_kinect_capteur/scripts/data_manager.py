#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Global variable to hold the point cloud data
point_cloud = None

# Callback function for processing the PointCloud2 data
def pointcloud_callback(msg):
    global point_cloud_pub

    try:
        modified_points = []

        for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            modified_points.append([x, y, z])
        
        rospy.loginfo(f"{modified_points}")
        return np.array(modified_points)


    except Exception as e:
        rospy.logerr("Failed to process point cloud: %s" % str(e))

# # Function to visualize the point cloud
# def visualize_point_cloud():
#     global point_cloud

#     if point_cloud is None:
#         rospy.loginfo("Waiting for point cloud data...")
#         return

#     try:
#         # Create an Open3D visualizer
#         vis = o3d.visualization.Visualizer()
#         vis.create_window("3D Point Cloud Visualization")

#         # Add the point cloud to the visualizer
#         vis.add_geometry(point_cloud)

#         # Run the visualizer
#         vis.run()

#         # Close the visualizer window
#         vis.destroy_window()

#     except Exception as e:
#         rospy.logerr("Failed to visualize point cloud: %s" % str(e))

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("kinect_pointcloud_visualizer_ocd", anonymous=True)

    # Subscribe to the /camera/depth/points topic
    rospy.Subscriber("/camera/depth/points_black", PointCloud2, pointcloud_callback)
    rospy.spin()
