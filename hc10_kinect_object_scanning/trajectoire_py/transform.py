#!/usr/bin/env python3


import rospy
import tf2_ros
import geometry_msgs.msg

def transform_point():
    rospy.init_node('transform_point_node')

    # Create a tf2 buffer and listener to listen for transforms
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Wait for transform to be available (between 'target_frame' and 'source_frame')
    source_frame = "base_link"
    target_frame = "map"
    try:
        # Wait for the transform between 'source_frame' and 'target_frame'
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))

        # Define the point in the 'source_frame'
        point_stamped = geometry_msgs.msg.PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.point.x = 1.0
        point_stamped.point.y = 2.0
        point_stamped.point.z = 3.0

        # Transform the point to the 'target_frame'
        transformed_point = tf_buffer.transform(point_stamped, target_frame)

        rospy.loginfo(f"Transformed Point: {transformed_point.point.x}, {transformed_point.point.y}, {transformed_point.point.z}")

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to transform point")


transform_point()
