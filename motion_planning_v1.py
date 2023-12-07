from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import math
import subprocess
import ast
from random import random
from rrt_functions import *

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from pybullet_tools.utils import remove_body, get_joint_positions, get_joint_position, get_bodies, get_body_name, quat_from_euler, create_attachment, plan_cartesian_motion, wait_for_duration, draw_pose, dump_body, set_renderer

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


#############################################################

def generate_rrt(world, goal, max, percent):
    joints = get_movable_joints(world.robot)
    obstacles = []
    for b in get_bodies():
        if world.robot != b and b!= 3: # and b!= 4:
            obstacles.append(b)
    dist_func = get_dist_func(world.robot, world.arm_joints)
    sample_func = get_sample_func(world.robot, world.arm_joints)
    # step_func not reliant on world and bodies. Already hardcoded
    collision_func = get_collision_func(world.robot, world.arm_joints, obstacles)

    # get start configurations
    start = (get_joint_positions(world.robot, joints))[9:16]

    # perform RRT
    set_renderer(enable=False) # do not visualize the RRT collision checking
    states = None
    while states == None:
        # print("[*] Thinking...")
        states = basic_rrt(start, goal, dist_func, step_func, sample_func, collision_func, None, max, percent)
    #if states is not None:
        # print("[+] Path found!")
    #else:
        # print("[-] RRT failed!")
    # reset initial configuration
    set_joint_positions(world.robot, world.arm_joints, start)
    set_renderer(enable=True)

    # execute RRT plan
    for state in states:
        set_joint_positions(world.robot, world.arm_joints, state)
        wait_for_duration(1e-1)













def main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())
    testFunction()

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

    obstacles = []
    for b in get_bodies():
        if world.robot != b and b != 3: # and b!= 5 and b!= 4:
            obstacles.append(b)
        # obstacles.append(b)
    # print(obstacles)

    for i in range(100):
        goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
        set_joint_positions(world.robot, world.base_joints, goal_pos)


        for obstacle in obstacles:
            # print(get_body_name(obstacle), get_body_name(obstacle))
            if pairwise_collision(world.robot,obstacle): # check if body has hit any obstacles
                print("I hit ", obstacle,get_body_name(obstacle))
    #             print(get_body_name(obstacle))
                

        if (i % 30 == 0):
            wait_for_user()


    goal = (-0.7485783467743256, 0.6000518546809359, 0.1886607294919161, -1.4924298630276667, -0.12165437028136393, 2.081177397934699, 2.6089407853951716)

    generate_rrt(world, goal, 200, 0.3)
    wait_for_user()
    goal = (0.01064202542796632, 1.4, 0.018605815479346265, -0.5, -0.05907068003938907, 3.1, 0.7906325747027143)

    generate_rrt(world, goal, 200, 0.3)

    # for i in range(2):
    #     print('Iteration:', i)
    #     conf = sample_fn()map_path(sample_step_node.path())  
    #     set_joint_positions(world.robot, world.arm_joints, conf)
    #     wait_for_user()
    #     ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    #     start_pose = get_link_pose(world.robot, tool_link)
    #     end_pose = multiply(start_pose, Pose(Point(z=1.0)))
    #     for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
    #         conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
    #         if conf is None:
    #             print('Failure!')
    #             wait_for_user()
    #             break
    #         set_joint_positions(world.robot, ik_joints, conf)
    print("Going to operate the base without collision checking")


    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
