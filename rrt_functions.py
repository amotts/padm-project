from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import math
import subprocess
import ast
from random import random

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from pybullet_tools.utils import remove_body, get_joint_positions, get_joint_position, get_bodies, get_body_name, quat_from_euler, create_attachment, plan_cartesian_motion, wait_for_duration, draw_pose, dump_body

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly


######################################################################
# RRT base code below
#####################################################################

class rrtNode(object):

    def __init__(self, state, parent):
        self._parent = parent
        self._state = state


    def path(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node._parent
        return sequence[::-1]

def map_path(nodes):
    return list(map(lambda n: n._state, nodes))



def basic_rrt(start, goal, dist_func, step_func, sample_func, collision_func, goal_func, max_steps=500, percent_goal=0.33):
    # Step 1: Check goal and start locations for collisions with obstacles
    if collision_func(goal)or collision_func(start): 
        print("Invalid Start of Goal")
        return(None)
    # Step 2:create list of rrt nodes
    nodes = [rrtNode(start, None)]
    # Step 3: Create limiting loop for number steps taken before declaring failure
    steps_taken=0
    while steps_taken < max_steps:
        # Step 4: Get sample point
        r = random()
        if r < percent_goal:
            sample_point = goal
        else:
            sample_point = sample_func()
        # Step 5: Find nearest node by using argmin of distance function
        closest = nodes[np.argmin([dist_func(n._state, sample_point) for n in nodes])]
        # Step 6: get point step distance along line
        sample_step = step_func(sample_point, closest._state)
        # Step7: check point for collision with collision function
        if collision_func(sample_step):
            continue
        # Step 8: add to queue and check if is goal state. if so return path
        sample_step_node = rrtNode(sample_step, closest)
        nodes.append(sample_step_node)
        if goal_func == None:
            if sample_step_node._state == goal:
                return map_path(sample_step_node.path())
        else:
            if goal_func(sample_step_node._state, goal):
                # print(len(nodes))
                return map_path(sample_step_node.path())           
        steps_taken += 1

    #if solution is not found within limit of steps
    print("Failed: Solution not found within step limit")
    return(None)

#####################################################################
# Required RRT helper functions
####################################################################

def get_sample_func(body, joints, custom_limits={}, **kwargs): # provided in limited example
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn


def get_dist_func(body, joints):
    def func(state1, state2):
        # take geometric distance in n dimension where n is length of state vectors
        return(math.sqrt(sum((state1[i]-state2[i])**2 for i in range(len(state1)))))
    return(func)

def get_collision_func(body, joints, obstacles):
    def func(state):
        set_joint_positions(body, joints, state) #set the position to be at state
        for obstacle in obstacles:
            if pairwise_collision(body,obstacle): # check if body has hit any obstacles
                print("I hit ", obstacle, get_body_name(obstacle))
                return True
        return False
    return(func)

def step_func(state1, state2, step_size = 0.5):
    #state1 is sample, state2 is closest node
    #linearly scale each component of the vector by the scale factor beyond the step size    
    dist_i = math.sqrt(sum((state1[i]-state2[i])**2 for i in range(len(state1))))
    if dist_i < step_size: 
        new_state = ([(state1[i] - state2[i]) / dist_i * step_size + state2[i] for i in range(len(state2))])
        return(new_state)
    else:
        return(state1)


def testFunction():
    print("Import of functions worked")
    return