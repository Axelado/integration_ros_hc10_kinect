#!/usr/bin/env python3
import numpy as np

A = np.array([0,0,0]) #position actuel      
B = np.array([1,20,0]) #centre ellipse/objet
C = np.array([-6,1,0]) #position suivante    

def calcul_repere(A,B,C):
    A=np.array(A)
    B=np.array(B)
    C=np.array(C)
    AB = B-A
    AB_norm = np.linalg.norm(AB)
    AC = C-A

    z = AB/AB_norm
    y = np.cross(AB,AC)/np.linalg.norm(np.cross(AB,AC))
    x = np.cross(y,z)/np.linalg.norm(np.cross(y,z))
    return [x,y,z]


def calcul_matrice_transf(repere,A):
    
    monde = np.array([[1,0,0],[0,1,0],[0,0,1]])
    R1 = repere
    R2 = monde

    R = np.dot(R2,np.linalg.inv(R1))

    return R


def quaternion(R):
    w = 1/2 * np.sqrt(1+R[0][0]+R[1][1]+R[2][2])
    x = (R[2][1]-R[1][2])/4*w
    y = (R[0][2]-R[2][0])/4*w
    z = (R[1][0]-R[0][1])/4*w
    return [x,y,z,w]

def calcul_quaternion(A,B,C):
    repere = calcul_repere(A,B,C)
    R = calcul_matrice_transf(repere,A)
    quat = quaternion(R)
    return quat


calcul_quaternion(A,B,C)







