from motion_planning_v1 import *
from rrt_functions import *



#default settings for RRT
MAX_ITER = 200
PERCENT_GOAL = 0.33
STEP_SIZE = 0.1
DRAWER_DIST = 0.3
OBSTACLES_TO_IGNORE = [3, 4, 5]

#World Setup Settings
PLANNER = "activity_planner_basic"
DOMAIN = "pddl_domain.pddl"
PROBLEM = "pddl_problem_simple.pddl"

#Action Dictionary
# Actions: open_door, close_door, grab_thing, release_thing, move_object, move_arm
action_list = ["open_door", "close_door", "grab_thing", "release_thing", "move_object", "move_arm"]


def blank_func(a=None, b=None,c=None, d=None, e=None):
    return()

ACTION_FUNCTIONS = dict()

ACTION_FUNCTIONS["open_door"] = generate_rrt
ACTION_FUNCTIONS["close_door"] = generate_rrt
ACTION_FUNCTIONS["grab_thing"] = blank_func
ACTION_FUNCTIONS["release_thing"] = blank_func
ACTION_FUNCTIONS["move_object"] = generate_rrt_w_object
ACTION_FUNCTIONS["move_arm"]= generate_rrt


GOAL_POSES = dict()
for act in action_list:
    GOAL_POSES[act]=dict()


GOAL_POSES["open_door"]["drawer"] = (0.5561482417440563, -1.0251061869539213, -1.9609893323598375, -2.2014717805836304, -0.2589333796789931, 2.6568843331727328, -0.6773986969906863)

GOAL_POSES["close_door"]["drawer"] = (1.3468115429378031, -1.1855967232420714, -2.2134869146883633, -1.7960922256926946, -0.2526004518888567, 2.928824959281513, -0.42977342085737813)

GOAL_POSES["move_arm"]["drawer"]=(1.3468115429378031, -1.1855967232420714, -2.2134869146883633, -1.7960922256926946, -0.2526004518888567, 2.928824959281513, -0.42977342085737813)
GOAL_POSES["move_arm"]["stove"]=(0.01064202542796632, 1.4, 0.018605815479346265, -0.5, -0.05907068003938907, 3.1, 0.7906325747027143)
GOAL_POSES["move_arm"]["counter"]=(-0.7485783467743256, 0.6000518546809359, 0.1886607294919161, -1.4924298630276667, -0.12165437028136393, 2.081177397934699, 2.6089407853951716)

GOAL_POSES["move_object"]["drawer"] = (1.0693663316413473, -0.9732932697997243, -2.1096123553694426, -2.2952996143893194, -0.2785314178255012, 3.1121042557799767, -0.3822184282329072)
GOAL_POSES["move_object"]["counter"] = (1.880353502615019, -1.348320286822579, -2.0462322486967235, -0.9027585788708357, 0.0898294783293007, 2.654121236088206, -0.5745334436812624)

GOAL_POSES["grab_thing"]["spam"] = None
GOAL_POSES["grab_thing"]["sugar"] = None
GOAL_POSES["release_thing"]["spam"] = None
GOAL_POSES["release_thing"]["sugar"] = None


OBJECT_DICT = dict()
OBJECT_DICT["spam"]= 5
OBJECT_DICT["sugar"] = 4
OBJECT_DICT[None]= None

DOOR_DICT=dict()
DOOR_DICT["drawer"] = 56
DOOR_DICT[None] = None