#!/usr/bin/env python3

import rospy
import json
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

point_cloud_pub = None
output_file_path = 'output.json'

def pointcloud_callback(msg):
    global point_cloud_pub

    try:
        modified_points = []

        min_z = -1.0
        max_z = 5.0  

        for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            
            if min_z <= z <= max_z:
                black_rgb = 0
                modified_points.append({
                    'x': x,
                    'y': y,
                    'z': z,
                    'rgb': black_rgb
                })

        with open(output_file_path, 'w') as file:
            json.dump(modified_points, file)
        
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id

        black_pointcloud_msg = pc2.create_cloud(header, msg.fields, [(p['x'], p['y'], p['z'], p['rgb']) for p in modified_points])
        point_cloud_pub.publish(black_pointcloud_msg)

    except Exception as e:
        rospy.logerr("Failed to process point cloud: %s" % str(e))

if __name__ == "__main__":
    rospy.init_node("kinect_pointcloud_modifier", anonymous=True)
    point_cloud_pub = rospy.Publisher("/camera/depth/points_black", PointCloud2, queue_size=10)
    rospy.Subscriber("/camera/depth/points", PointCloud2, pointcloud_callback)
    rospy.spin()
