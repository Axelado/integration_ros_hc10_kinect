#!/usr/bin/env python3

#import fonction creer
from ellipse import ellipse
from orientation import calcul_quaternion
import tf2_geometry_msgs
import tf2_ros
import rospy
from geometry_msgs.msg import Point, PointStamped

#api movit (bouger le robot)
import moveit_commander
from geometry_msgs.msg import Pose

def get_pose():
    

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
            trans = tfBuffer.lookup_transform( target_link,ref_link, rospy.Time(0))
            
            # Extraire la position
            position = trans.transform.translation
            rospy.loginfo("Position: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(position.x, position.y, position.z))
            trouve = True
            print("Trouvé")
            return trans
           
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transformation non trouvée")
        
    
    # rate.sleep()


[x,y],h,centre_ellipse = ellipse()#creation des points de la trajectoire 
print([x,y],h,centre_ellipse)
groupe_name = "hc10_arm"#nom du bras robotique
move_groupe = moveit_commander.MoveGroupCommander(groupe_name)#creation du commander afin de commander le bras
target_Pose = Pose()#creation de la target pose dans le bon format

trajectoire = []

# creation d'une liste des points à atteindre
for z in h: # itération sur les différentes hauteur
    for i in range(len(x)): # iteration des different points 2D de l'ellipse [x,y]
        sensor_frame = get_pose()
        point = PointStamped()
        point.point.x = x[i]
        point.point.y = y[i]
        point.point.z = z
    
        output = tf2_geometry_msgs.do_transform_point(point, sensor_frame)
        
        trajectoire.append([output.point.x, output.point.y, output.point.z])
        

for i,[x,y,z] in enumerate(trajectoire): # itération sur les points a atteindre
    if i < len(trajectoire)-1:
        print("ii",i)
        target_Pose.position.x=x
        target_Pose.position.y=y
        target_Pose.position.z=z
        print("traj i", trajectoire[i])
        A = trajectoire[i]  #position actuel
        B = [centre_ellipse[0],centre_ellipse[1],z]  #centre ellipse/objet
        C = trajectoire[i+1] #position suivante
        [ox,oy,oz,ow] = calcul_quaternion(A,B,C)
        target_Pose.orientation.x=ox
        target_Pose.orientation.y=oy
        target_Pose.orientation.z=oz
        target_Pose.orientation.w=ow

        move_groupe.set_pose_target(target_Pose) # donne a moveit la position goal

        plan = move_groupe.go(wait=True) #le bras bouge vers la position de destination

        move_groupe.stop() # arrete le mouvement
        move_groupe.clear_pose_targets() # supprime la position goal sur moveit

