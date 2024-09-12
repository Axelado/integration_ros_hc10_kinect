#!/usr/bin/env python3

#import fonction creer
from ellipse import Ellipse_Create
from orientation import calcul_quaternion
#import listener
import rospy
from moveit_msgs.msg import MoveGroupActionGoal
#api movit (bouger le robot)
import moveit_commander
from geometry_msgs.msg import Pose
#var globale
resume = False
nuage_pt = None
#ecoute du topic
def callback(data):
    resume = True
    nuage_pt = data.goal
     
def listener(TopicName):
    # Init the listener
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber(TopicName, MoveGroupActionGoal, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

topic_capt = "/move_group/goal" #nom du topic a ecouter
#nuage_pt = listener(topic_capt) #recuperation des valeurs du capteur
nuage_pt = [[2,2,2],[2,-3,5],[6,5,7],[-9,5,28],[-10,5,7],[15,2,9],[4,-8,2]]
pt,h,centre_ellipse = Ellipse_Create(nuage_pt)#creation des points de la trajectoire 
print(pt,h)
groupe_name = "hc10_arm"#nom du bras robotique
move_groupe = moveit_commander.MoveGroupCommander(groupe_name)#creation du commander afin de commander le bras
target_Pose = Pose()#creation de la target pose dans le bon format

trajectoire = []


for z in h: # itération sur les différentes hauteur
    for [x,y] in pt: # iteration des different points 2D de l'ellipse [x,y,ox,oy,oz,ow]
        trajectoire.append([x,y,z])

for i,[x,y,z] in enumerate(trajectoire):
    if i < len(trajectoire):
        target_Pose.position.x=x
        target_Pose.position.y=y
        target_Pose.position.z=z
        A = trajectoire[i]
        B = centre_ellipse
        C = trajectoire[i+1]
        [ox,oy,oz,ow] = calcul_quaternion(A,B,C)
        target_Pose.orientation.x=ox
        target_Pose.orientation.y=oy
        target_Pose.orientation.z=oz
        target_Pose.orientation.w=ow

        move_groupe.set_pose_target(target_Pose) # donne a moveit la position goal

        plan = move_groupe.go(wait=True) #le bras bouge vers la position de destination

        move_groupe.stop() # arrete le mouvement
        move_groupe.clear_pose_targets() # supprime la position goal sur moveit
        while resume == False: #permet d'attendre que le capteur a recupérer les information
            continue
        resume = False


