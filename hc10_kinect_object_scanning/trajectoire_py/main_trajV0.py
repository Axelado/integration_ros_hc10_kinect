#!/usr/bin/env python3

#import fonction creer
from ellipse import ellipse
from orientation import calcul_quaternion

#api movit (bouger le robot)
import moveit_commander
from geometry_msgs.msg import Pose

pt,h,centre_ellipse = ellipse()#creation des points de la trajectoire 
print(pt,h,centre_ellipse)
groupe_name = "hc10_arm"#nom du bras robotique
move_groupe = moveit_commander.MoveGroupCommander(groupe_name)#creation du commander afin de commander le bras
target_Pose = Pose()#creation de la target pose dans le bon format

trajectoire = []

# creation d'une liste des points à atteindre
for z in h: # itération sur les différentes hauteur
    for [x,y] in pt: # iteration des different points 2D de l'ellipse [x,y]
        trajectoire.append([x,y,z])

for i,[x,y,z] in enumerate(trajectoire): # itération sur les points a atteindre
    if i < len(trajectoire):
        target_Pose.position.x=x
        target_Pose.position.y=y
        target_Pose.position.z=z
        A = trajectoire[i]  #position actuel
        B = centre_ellipse  #centre ellipse/objet
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
        while resume == False: #permet d'attendre que le capteur a recupérer les information
            continue
        resume = False

