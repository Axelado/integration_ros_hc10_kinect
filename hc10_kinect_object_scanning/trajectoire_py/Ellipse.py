import math
import numpy as np
import matplotlib.pyplot as plt


nua = [[2,2,2],[2,-3,5],[6,5,7],[-9,5,28],[-10,5,7],[15,2,9],[4,-8,2]]


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

    #Détermination des différents niveaux autour desquels on effectuera une révolution

    num_levels = math.ceil(MaxZ / 10)  # Arrondir au prochain entier si c'est un flottant    
    # Générer les niveaux de Z en fonction de MaxZ et du nombre de niveaux
    levels = np.linspace(0, MaxZ, num_levels + 1)[1:]


    return [x,y],levels

def closest(nuageinLocal):
    points = np.array(nuageinLocal)
    MaxZ = points[np.argmax(points[:, 2])]
    return MaxZ[2]
    


[x,y],level = Ellipse_Create(nua,1.5)
