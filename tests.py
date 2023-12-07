from motion_planning_v1 import rrtNode, map_path, basic_rrt

import numpy as np
import math
import subprocess
import ast
from random import random


#################################
# 2-D cartesian t for basic_rrt to find a path from start to goal

start = (0,0)
goal = (5,4)
def dist_func(x,y):
    return(math.sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2))

def sample_func():
    return(6*random(),6*random())

def collision_func(x):
    if x[0] < 3 and x[1] > 3:
        return True
    else:
        return False
    #return(False)

def take_step(pt2, pt1, step=0.25):
    if ((pt2[1]-pt1[1])**2+(pt2[0]-pt1[0])**2 <= (step)**2):
        return (pt2)
    else:
        theta = np.tan((pt2[1]-pt1[1])/(pt2[0]-pt1[0]))
        x_new = pt1[0] + step*np.cos(theta)
        y_new = pt1[1] + step*np.sin(theta)
    return((x_new, y_new))

def goal_func(x, y): 
    if (math.sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2) < 0.25):
        return(True)
    else:
        return(False)




path = basic_rrt(start, goal, dist_func, take_step, sample_func, collision_func, goal_func, 500, 0.33)

print(path)
if path != None:
    print(len(path))

########################################