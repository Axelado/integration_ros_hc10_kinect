#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose
import moveit_commander
import tf.transformations as tft
import tf2_ros
import geometry_msgs.msg

alpha = 0
points = []
safety_distance = 0.20
update_rate = 10  # Hz

group_name = "hc10_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
target_pose = Pose()

def transform_point_to_world(point, frame_id):
    listener = tf2_ros.Buffer()
    tf2_ros.TransformListener(listener)
    
    try:
        transform = listener.lookup_transform('world', frame_id, rospy.Time(0), rospy.Duration(1.0))
        
        # Convert point to a PointStamped
        point_camera = geometry_msgs.msg.PointStamped()
        point_camera.header.frame_id = frame_id
        point_camera.header.stamp = rospy.Time(0)
        point_camera.point.x = point[0]
        point_camera.point.y = point[1]
        point_camera.point.z = point[2]
        
        # Transform the point to the world frame
        point_world = listener.transformPoint('world', point_camera)
        
        return [point_world.point.x, point_world.point.y, point_world.point.z]
    
    except Exception as e:
        rospy.logerr("Failed to transform point: %s" % str(e))
        return point

def next_point(center_point, safety_distance, height, safety_x=True):
    global alpha, move_group, target_pose
    
    epsilon = 0.05
    alpha += (np.pi / 2) * 0.1
    
    start = np.array(center_point)
    end = start + np.array([safety_distance, 0, -epsilon])
    
    r = np.linalg.norm(start - end) / 2
    
    next_points = [start[0] + r * np.cos(alpha), 
                   start[1] * (1 - np.cos(alpha)), 
                   start[2] + r * np.sin(alpha)]
    
    next = np.array(next_points)
    
    # Add security distance
    security_distance = 0.20
    next += np.array([security_distance, security_distance, security_distance])
    
    target_pose.position.x = next[0]
    target_pose.position.y = next[1]
    target_pose.position.z = next[2]
    
    direction_vector = next - start
    direction_vector /= np.linalg.norm(direction_vector)
    
    z_axis = np.array([0, 0, 1])
    axis = np.cross(z_axis, direction_vector)
    angle = np.arccos(np.dot(z_axis, direction_vector))
    
    if np.linalg.norm(axis) > 0:
        axis /= np.linalg.norm(axis)
        quaternion = tft.quaternion_about_axis(angle, axis)
    else:
        quaternion = [0, 0, 0, 1]
    
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]
    
    move_group.set_pose_target(target_pose)
    plan = move_group.go(wait=True)
    
    move_group.stop()
    move_group.clear_pose_targets()
    
    return alpha

def pointcloud_callback(msg):
    global alpha, points, safety_distance
    
    points = []
    
    try:
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            transformed_point = transform_point_to_world([x, y, z], msg.header.frame_id)
            points.append(transformed_point)
        
        points = np.array(points)
        
    except Exception as e:
        rospy.logerr("Failed to process point cloud: %s" % str(e))

def timer_callback(event):
    global alpha, points, safety_distance
    
    if len(points) == 0:
        return
    
    center_point = np.mean(points, axis=0)
    width_x = max(points[:, 0])
    width_y = max(points[:, 1])
    way = True if width_x > width_y else False
    
    safety_distance += max([width_y, width_x])
    
    alpha = next_point(center_point, safety_distance, way)
    
    if alpha >= np.pi/2:
        rospy.signal_shutdown()

if __name__ == "__main__":
    rospy.init_node("point_cloud_retrieval", anonymous=True)
    rospy.Subscriber("/camera/depth/points_black", PointCloud2, pointcloud_callback)
    
    # Create a timer to call the timer_callback function periodically
    rospy.Timer(rospy.Duration(1.0 / update_rate), timer_callback)
    
    rospy.spin()
