# PADM Final Project
## Overview

This is the repository for all files relating to the my final project for Principles of Autonomy and Design Fall 2023. There were three sections, broadly representing the three major topics covered in the class. The first part was to implement an activity planner that would take in a description of the situation and output a valid list of actions that would result in the goal state. The second part was to take the activity plan generated in the previous step and implement a motion planner that would execute the activity plan on a Franka Panda Arm robot and visualize it in the provided simulator. The third and last part was to demonstrate a trajectory optimization on one of the arm movements using constraint optimization.

The problem was posed as a kitchen with four types of locations: stovetops, cabinets, drawers, and countertops. Objects are placed within these locations in an initial configuration and the goal configuration requires them to be moved to pre-specified locations. The given conditions were:
* Initial condition: Box of sugar on the stovetop and spam can on the counter
* Goal condition: Box of sugar on the countertop and spam can in the drawer

Below is documenation of the approach I took, descriptions of important files, and notes on sucess/limitations of these implementations.

## Part 1: Activity Planning 
**PDDL Domain and Problem**

To describe the kitchen enviornment, a domain needed to be established. I used the reccomended PDDL format and `pddl-parser` libary. I attempted to make the domain as unrestricted as possible and allowed for any number of robots, drawers, cabinets, stoves, counters, and objects to be present. I made the assumptions however that each robot could only hold one object at a time and each counter space could only hold one object at a time. Additionally, I assumed that moving the arm with or without an object was a seperate action from closing or opening the gripper. For simplicity sake, I also encoded the assumption that all locations could be opened or closed. However to better reflect reality, all non-closeable locations will start open thus there should never be a need for their status to be changed in a optimal activity plan. Possible actions were written as `OPEN_DOOR`, `CLOSE_DOOR`, `GRAB_THING`, `RELEASE_THING`, `MOVE_OBJECT`, and `MOVE_ARM`. Most actions output with the applicable values from the format `Action Robot Current_location End_location Object`

`pddl_problem.pddl` represents a problem statement that has three of each cabinets, drawers, counters, and two stoves as a demostration of the scalability of the domain. For the purpose of the project, `pddl_problem_simple.pddl` represents a world with only one countertop, stove, cabinet, and drawer as refelected in the simulator.

**Planner**

I implemented a Best-First planner using a `PriorityQueue` similar to that from the course psets. I used a Fast Forward heuristic that evaluated each option based on the length of path it would take the relaxed problem to reach the goal state. 

**Results**

This planner sucessfully solves the problem and ouptuts a list of action which achieve the goal state. However, this plan included superflous steps that, on inspection, were not necessary in order to optimally achieve the goal state. This is due to the fact that of the large number of possible actions within the releaxed problem, a large number of them all have the same FF cost and thus are inserted at the top of the queue. The example output for the large kitchen `pddl_problem.pddl` is below:

```Shell
python -B -m activity_planner_best_first pddl_domain.pddl pddl_problem.pddl
```
```Shell
Time: 27.194578409194946s
plan:
grab_thing robot c-counter spam
move_object robot c-counter r-stove spam
release_thing robot r-stove spam
move_arm robot r-stove l-stove
grab_thing robot l-stove sugar
move_object robot l-stove c-counter sugar
release_thing robot c-counter sugar
move_arm robot c-counter r-stove
grab_thing robot r-stove spam
move_object robot r-stove r-counter spam
release_thing robot r-counter spam
move_arm robot r-counter g-drawer
open_door robot g-drawer
move_arm robot g-drawer r-counter
grab_thing robot r-counter spam
move_object robot r-counter r-stove spam
release_thing robot r-stove spam
move_arm robot r-stove c-cabinet
open_door robot c-cabinet
move_arm robot c-cabinet r-stove
grab_thing robot r-stove spam
move_object robot r-stove r-counter spam
release_thing robot r-counter spam
move_arm robot r-counter r-drawer
open_door robot r-drawer
move_arm robot r-drawer r-counter
grab_thing robot r-counter spam
move_object robot r-counter r-drawer spam
release_thing robot r-drawer spam
```
Going forward, I would need to revise how the best option is chosen and take into consideration the total length of the path to the goal with an attempt to minimize the number of actions taken to reach the goal. However, when used on the much smaller `pddl_problem_simple.pddl`, the soulution outputted is equavalent to that provided by the included example solver and by inspection, is the optimal solution.
```Shell
Python -B -m activity_planner_best_first pddl_domain.pddl pddl_problem_simple.pddl
```
```Shell
Time: 0.27944445610046387s
plan:
move_arm robot counter drawer
open_door robot drawer
move_arm robot drawer counter
grab_thing robot counter spam
move_object robot counter drawer spam
release_thing robot drawer spam
move_arm robot drawer stove
grab_thing robot stove sugar
move_object robot stove counter sugar
release_thing robot counter sugar
```

**Files**

`activity_planning_scratch.txt`: document containing scratchwork and notes for this portion of the project

`pddl_domain.pddl`: establish a kitchen domain with robots, locations, and things. Locations include countertops, stovetops, drawers, cabinets. Actions are to open or close locations or move object from one location to another

`pddl_problem.pddl`: establish a problem within a kitchen domain with two stoves, 3 counters, 3 cabinets, and 3 drawers, and a gripper. 

`pddl_problem_simple.pddl`: simple version of the pddl problem but with only one stove, counter, cabinet, and drawer, to better model the kitchen provided

`activity_planner_basic.py`: activity planner using a Breadth-First Search method by adding new satisifed actions to the back of the queue. 

`activity_planner_best_first.py`: a functioning activity planner using a Best First Search method similar to Pset 2. Cost is calculated by a FF ignorning any delete effects and negative preconditions. 


## Part 2: Motion Planning

Did some file reorganization and upkeep. Removed the repository for the padm-project-2023f so the files could be added to my repository. Removed redunadant nestled directories to shorten paths and better allow importing from files and tools provided

motion_planner_scratch: Planning notes, observations, and performance comments about motion planning files and code

motion_planning_v1: Programmed with ability to drive robot to location, move arm, move arm with held object. Open/close drawer accomplished by adding drawer motion to regular arm motion. Action plan generated from any planner, domain, and problem files inputted in def main().

action_planner.py: contains function to run the python command to execute the planner on the domain and problem. Provides plan as a list. Each list element is a list of the output for that action.

rrt_functions.py: Implimented code for basic rrt including rrt nodes and rrt protocol for basic rrt. After testing, decided to include a percentage of samples to be the goal to speed things along. Included optional goal_function to allow for future flexibility in what is considered reaching the goal. Testing in 2D cartesian form showed over a 60% reduction in number of nodes explored to find goal. necessary functions are written but not tested

tests.py: file for creating test cases to test individual function performance:
    Contents:
    - 2D cartesian test for basic RRT

Shortcomings:
    - Motion is not smooth for open drawer
    - poses get messed up when picking up
    - positions for each object must be hardcoded into dictionaries base on actions
    - not fully flushed out collision checking for not hitting object when object is held
