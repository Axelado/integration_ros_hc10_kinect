#!/usr/bin/env python3
from pickle import NONE
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math
import matplotlib.pyplot as plt


#nua = [[2,2,2],[2,-3,5],[6,5,7],[-9,5,28],[-10,5,7],[15,2,9],[4,-8,2]]
coord = None


def Ellipse_Create(nuage,scale=1):

    points = np.array(nuage)
    
    
    #Détermination des points limites
    
    MaxX = points[np.argmax(points[:, 0])]
    MinX = points[np.argmin(points[:, 0])]
    MaxY = points[np.argmax(points[:, 1])]
    MinY = points[np.argmin(points[:, 1])]
    MaxZ = points[np.argmax(points[:, 2])]
    MaxZ = MaxZ[2]  

    #Création de l'ellipse sur un plan en 2 dimensions
    centerx = (MaxX[0]+MinX[0])/2
    centery = (MaxY[1]+MinY[1])/2

    a = abs(MaxX[0]-MinX[0])/2
    b = abs(MaxY[1]-MinY[1])/2

    t = np.linspace(0,2*np.pi,12)

    x =  centerx + a*np.cos(t)*scale
    y = centery + b*np.sin(t)*scale

    #Déterminationcoord des différents niveaux autour desquels on effectuera une révolution

    num_levels = math.ceil(MaxZ / 10)  # Arrondir au prochain entier si c'est un flottant    
    # Générer les niveaux de Z en fonction de MaxZ et du nombre de niveaux
    levels = np.linspace(0, MaxZ, num_levels + 1)[1:]


    return [x,y],levels,[centerx,centery]

def closest(nuageinLocal):
    points = np.array(nuageinLocal)
    MinZ = points[np.argmin(points[:, 2])]
    return MinZ[2]


def pointcloud_callback(msg):
    global point_cloud_pub,coord

    try:
        modified_points = []

        for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            modified_points.append([x, y, z])
        
        rospy.loginfo(f"{modified_points[:2]}")
        
        coord = Ellipse_Create(modified_points,1.5)
        # plt.plot(x,y)
        # plt.show()


    except Exception as e:
        rospy.logerr("Failed to process point cloud: %s" % str(e))


def ellipse():
    global coord
    # Initialize the ROS node
    rospy.init_node("kinect_pointcloud_visualizer_ocd", anonymous=True)

    # Subscribe to the /camera/depth/points topic
    rospy.Subscriber("/camera/depth/points_black_transformed", PointCloud2, pointcloud_callback)
    while coord == None:
        continue
    tmp = coord
    coord = None
    return tmp

