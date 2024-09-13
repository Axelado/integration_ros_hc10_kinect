#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

def get_pose():
    rospy.init_node('get_link_6_pose', anonymous=True)

    # Créer un buffer et un listener pour tf2
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    target_link = 'base_link'
    ref_link = 'kinect_link'
    
    trouve = False
    rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    while(trouve==False):
        try:
            # Chercher la transformation de 'base_link' à 'link_6_l'
            trans = tfBuffer.lookup_transform(ref_link, target_link, rospy.Time(0))
            
            # Extraire la position
            position = trans.transform.translation
            rospy.loginfo("Position: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(position.x, position.y, position.z))
            trouve = True
            print("Trouvé")
            return position
            # # Extraire l'orientation (quaternion)
            # quaternion = (
            #     trans.transform.rotation.x,
            #     trans.transform.rotation.y,
            #     trans.transform.rotation.z,
            #     trans.transform.rotation.w
            # )
            
            # Extraire l'orientation (quaternion)
            quaternion = trans.transform.rotation
            rospy.loginfo("Orientation (quaternion): x: {:.2f}, y: {:.2f}, z: {:.2f}, w: {:.2f}".format(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w))
            
            # # Convertir en angles d'Euler (roll, pitch, yaw)
            # euler = euler_from_quaternion(quaternion)
            # rospy.loginfo("Orientation (rpy): Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(euler[0], euler[1], euler[2]))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transformation non trouvée")
        
    
    # rate.sleep()

if __name__ == '__main__':
    try:
        get_pose()
    except rospy.ROSInterruptException:
        pass
