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

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

######################################################################
# Povided setup above
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









def main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    wait_for_user()
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    print("Going to use IK to go from a sample start state to a goal state\n")
    for i in range(2):
        print('Iteration:', i)
        conf = sample_fn()
        set_joint_positions(world.robot, world.arm_joints, conf)
        wait_for_user()
        ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
        start_pose = get_link_pose(world.robot, tool_link)
        end_pose = multiply(start_pose, Pose(Point(z=1.0)))
        for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                print('Failure!')
                wait_for_user()
                break
            set_joint_positions(world.robot, ik_joints, conf)
    print("Going to operate the base without collision checking")
    for i in range(100):
        goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        if (i % 30 == 0):
            wait_for_user()
    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
