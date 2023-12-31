# PADM Final Project
## Overview

This is the repository for all files relating to the my final project for Principles of Autonomy and Design Fall 2023. There were three sections, broadly representing the three major topics covered in the class. The first part was to implement an activity planner that would take in a description of the situation and output a valid list of actions that would result in the goal state. The second part was to take the activity plan generated in the previous step and implement a motion planner that would execute the activity plan on a Franka Panda Arm robot and visualize it in the provided simulator. The third and last part was to demonstrate a trajectory optimization on one of the arm movements using constraint optimization.

The problem was posed as a kitchen with four types of locations: stovetops, cabinets, drawers, and countertops. Objects are placed within these locations in an initial configuration and the goal configuration requires them to be moved to pre-specified locations. The given conditions were:
* Initial condition: Box of sugar on the stovetop and spam can on the counter
* Goal condition: Box of sugar on the countertop and spam can in the drawer

Below is documenation of the approach I took, descriptions of important files, and notes on sucess/limitations of these implementations.

## Part 1: Activity Planning 
**PDDL Domain and Problem**

To describe the kitchen enviornment, a domain needed to be established. I used the reccomended PDDL format and `pddl-parser` libary. I attempted to make the domain as unrestricted as possible and allowed for any number of robots, drawers, cabinets, stoves, counters, and objects to be present. I made the assumptions however that each robot could only hold one object at a time and each counter space could only hold one object at a time. Additionally, I assumed that moving the arm with or without an object was a seperate action from closing or opening the gripper. For simplicity sake, I also encoded the assumption that all locations could be opened or closed. However to better reflect reality, all non-closeable locations will start open thus there should never be a need for their status to be changed in an optimal activity plan. Possible actions were written as `OPEN_DOOR`, `CLOSE_DOOR`, `GRAB_THING`, `RELEASE_THING`, `MOVE_OBJECT`, and `MOVE_ARM`. Most actions output with the applicable values from the format `Action Robot Current_location End_location Object`

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
**Sample Based Motion Planning**

In order to implement the activity plan generated in the previous part, I needed to implement a motion planner to direct the robot arm how to get from poin A to point B. As reccomended, I implemented a sample based RRT technique similar to that from the course pset. This generic RRT code accepts any start, goal, distance fucntion, step generating function, collision checking function, and goal checking function to allow for future changes to the specifics without needing to change the RRT code. After writing a 2-D cartesian test case in `tests.py`, I determined that the RRT was not efficient enough. To increase the likelihood of finding a valid path sooner, changed the sample function to randomly select the goal as the sample for a set percentage of the time, defaulting to 33%. In the 2D test case, this decreased the number of nodes expanded from ~170 to ~50 in order to find similar paths. The function returns a list of each state along the path.

**Integration with Simulated Robot**

To execute the RRT on the Franka arm, I chose to use the 7-dimensional joint space rather than 3 dimensional configuration space. Using joint space will allow for paths to be chosen easier that are possible given the limitation of the manipulator DOF. Limitations of this mean that motion will not be smooth or necessarily efficient, and manipulator will not move in a straight line in configuration space. Ths samples, collision, and goal checking use built-in functions provided in the course repo. The distance function takes a geometric distace within the 7-D joint space. The step taking function returns a linear step of specified size along the line between two states.

Several assumptions had to be hardcoded in. Since the RRT generated path is in joint space, the motion to open a drawer is not straight, thus I assumed that the gripper/drawer handle was flexible and thus the end effector did not need to stay exactly in front of the drawer and move in exclusively in the X direction. Additionally, the drawer was hardcoded to open linearly at the same time the effector undergoes its motion. The motion of the drawer and the effector are not inherently linked.

**Integration with Activity Planner**

The activity planner was run in python as part of the main() for the motion planner. The output of the activity planner was parsed and split into a list containing each element of the specific activity. To reduce redundancy in code, all outputs were augmented with None such that they would all have 5 components. Using this strategy allows the motion planner to be compatible with an number of locations, actions, objects, or doors/drawers as defined in `global_settings.py`. `ACTION_FUNCTIONS` dictionary linked the outputs of the activity planner to the corresponding action functions, with inputs corresponding to `GOAL_POSES`, `OBJECT_DICT`, and `DOOR_DICT` values. In a new world or with additional locations to be used, the corresponding joint space coordinates would need to be added into the `GOAL_POSES` dictionary for the robot to know where to go for each location. However, a limitation is that only one pose can be assigned for each location and it would have to be assumed that any corresponding objects could be grasped and released from that fixed location. Another assumption is that the base of the robot remains in the same pre-assigned location for all actions.

**Results**
Settings for world locations, the planner to use, and the problem and domains must be set in the `global_settings.py` file. Once set, the entire activity planner plus motion planner on the simulated robot can be run with the command:
```Shell
python -B motion_planning_v2.py
```
The terminal will then output plan before listing out each action as it performs it.
```Shell
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

Doing:  ['move_arm', 'robot', 'counter', 'drawer', None]
Press enter to continue
Doing:  ['open_door', 'robot', 'drawer', None, None]
Press enter to continue
Doing:  ['move_arm', 'robot', 'drawer', 'counter', None]
Press enter to continue
Doing:  ['grab_thing', 'robot', 'counter', 'spam', None]
Press enter to continue
Doing:  ['move_object', 'robot', 'counter', 'drawer', 'spam']
Press enter to continue
Doing:  ['release_thing', 'robot', 'drawer', 'spam', None]
Press enter to continue
Doing:  ['move_arm', 'robot', 'drawer', 'stove', None]
Press enter to continue
Doing:  ['grab_thing', 'robot', 'stove', 'sugar', None]
Press enter to continue
Doing:  ['move_object', 'robot', 'stove', 'counter', 'sugar']
Press enter to continue
Doing:  ['release_thing', 'robot', 'counter', 'sugar', None]
Press enter to continue
ALL TASKS COMPLETE
```
A videp of the full sequence can be seen in `motion_planning_complete.mp4`
[![](https://github.com/amotts/padm-project/blob/main/motion_planning.png?raw=true)](https://drive.google.com/file/d/1dpdtQMHbkYcIUfeZOTTVPraVkIlcMxXq/view?usp=sharing "Video Title")

or at [this link](https://drive.google.com/file/d/1dpdtQMHbkYcIUfeZOTTVPraVkIlcMxXq/view?usp=sharing)

The poses of objects is not well managed and jumps when the robot "grips" the object. Similarly, the pose when released stays at whatever positions it was in the manipulator and is not subject to gravity to set it down flat. In the future, an additional function to set the pose of the objects when picked up and released would alleviate this. As mentioned above, the drawer opening motion has been hardcoded and is not smooth. A link between the drawer and the effector could reduce this, however to obey the motion laws of the drawer, the arm would need to move in a straight line in configuration space.

**Files**

`motion_planner_scratch`: Planning notes, observations, and performance comments about motion planning files and code

`motion_planning_v1.py`: initial motion planning and implementation

`motion_planning_v2.py`: updated and simplified version of motion_planning

`global_settings.py`: File containing the planner, domain, problem, and all dictionaries to represent objects and locations in the world

`action_planner.py`: contains function to run the python command to execute the planner on the domain and problem. Provides plan as a list. Each list element is a list of the output for that action.

`rrt_functions.py`: Basic rrt including rrt nodes and generic rrt function. 

`tests.py`: 2D Test case for rrt function

## Part 3: Trajectory Optimization
**Constraint Programming**

I chose to optimize for the time it takes to make each movement using a first order motion model with velocity constraints. I defined a variable `X` for the 7-D joint space vector for h intervals. `Y` reprsents the 7-D velocity vector for h-1 intervals. `t` represents the length of time for each of the h-1 intervals. The goal was to minimize the sum of all of the time intervals. The constraints I encoded are:
- the trajectory starts at the start configuration
- the trajectory ends at the end configuration
- the velocity of all joints starts and ends at zero
- the magnitude of joint speeds stays below the maximum speed
- the distance between position steps is the joint velocity multiplied by the times step

An additional constraint that I would ideally encode would be the distance between any configuration and an obstacle remains above zero. However, due to time constraints, I did not implement.

The full constraint problem is encoded in `trajectory_optimization_time.py` can be posed as:

$$
\begin{aligned}
\min_{i} \text{ } \text{ } \Sigma t_i \\
\text{subject to } \text{ } \text{ } &X(i+1)=X(i)+Y(i)*t(i), \\
&X(0) = Start, \\
&X(h) = Goal, \\
&Y(0) = 0, \\
&Y(h) = 0, \\
&\forall i, \text{} 0 \leq t(i) \leq 0.1, \\ 
&\forall t, \text{ } |Y(i)| \leq Y_{max}
\end{aligned}
$$

**Solving and Results**

I used the built in Solve function from pydrake which selects the best solver after analyzing the constraints. In this case, it was `SNOPT`The solution of the `X` is converted into a list of configurations and the solution to `t` converted to a list of time step durations. I visualized the motion pathway using the same procedure as the RRT path except using the corresponding time step duration. I tested the motion on the arm movement from initial position to the drawer handle. The resulting motion path is significantly smoother and has far less shaking and rapid direction changes.

In `optimization_comparison.mp4`, the first trajectory to the drawer and back is with the constraint programming trajectory and the second is the RRT path from part2.

[![](https://github.com/amotts/padm-project/blob/main/optimization_comparison.png?raw=true)](https://drive.google.com/file/d/1gKbFvQdC6JEknKC0AvnA60UxVthJSov_/view?usp=sharing "Trajectory Comparison")

or at [this link](https://drive.google.com/file/d/1gKbFvQdC6JEknKC0AvnA60UxVthJSov_/view?usp=sharing)

**Files**

`trajectory_optimization_scratch`: Planning notes, observations, and performance comments about trajectory optimization files and code

`trajectory_optimization_time.py`: A function to run a constraint problem encoded in pydrake as a `MathematicalProgram` with the above constraints

`trajectory_optimization_test.py`: modidified version of `motion_planning_v1` which runs the trajectory to and from the drawer first with the optimized solver and then with the original RRT method


## Conclusion
This project has been an incredibly useful application of the concepts presented in class and briefly outlilned in the course psets. I was at first very confused and lost on where to even start, but over the course of this project I have gotten signifcantly more comfortable not only with the actual concepts of PDDL, Best First, RRT, Linear Programming, but also and most applicably with integrating existing libraries for simulation, modeling, and solving problems. I got experience learning how to read documentation and sort through libaries to find the key files and function. These skills I can carry over to my own research into marine vehicle autonomy.

A common difficulty throuhgout the project was dealing with the 7-dimensional joint space and being unable to easily mentally visualize the corresponding configuration space. Additionally, I struggled with poses of objects and did not completly suceed in maintaining the correct poses on during object motion in the motion planning. Part of what I learned was how to use "get_function" functions and use functions as inputs to other functions, allowing the same method and code to be applied in simpler test cases before expanding to higher degrees (e.g. `tests.py`).

### Resources Used

I completed this project on my own without a partner. I used refernce code and information gained from many sources including:
* Provided Project Repo Documentation
* Course Lectures and Psets
* Github repository from Pascal Spino and William Mitchell
* Pydrake documentation and tutorials





