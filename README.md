# PADM Final Project
Repository for all files relating to the final project for Principles of Autonomy and Design Fall 2023


## Part 1: Activity Planning 

activity_planning_scratch.txt: document containing scratchwork for determinining structure of PDDL enviornmentt to be created in new file

activity_planning_pddl_domain: establish a kitchen domain with locations and things. Locations include countertops, stovetops, drawers, cabinets. Options for actions are to open or close locations or move object from one location to another

activity_planning_pddl_problem: establish a problem within a kitchen domain with two stoves, 3 counters, 3 cabinets, and 3 drawers, and a gripper. All closeable locations are closed. Goal is occupy counter with sugar and red drawer with spam and have an empty gripper.

Notes: Currently, multiple actions are offered that would never be needed and are not representative of the real world (e.g. opening or closing a counter or stovetop). Since these predicates should have no bearing on any available actions and would never be on a shortest path to acchieve the goal, I don't expect them to pose any issues. They will increase computational load since they will always be an option to move the gripper to the location and change the state of the door, but I will deal with that issue if it comes up.

activity_planner_basic: a functional activity planner using a Breadth-First Search method by adding new satisifed actions to the back of the queue. Modeled after the BFS used in pset 1 using syntax from the
PDDL-parser. Run time is approx 1 sec
    runs with: python -B -m activity_planner_basic /home/amotts/Documents/MIT_FA23/padm-project/activity_planning_pddl_domain.pddl /home/amotts/Documents/MIT_FA23/padm-project/activity_planning_pddl_problem.pddl

activity_planner_best_first: a functioning activity planner using a Best First Search method similar to Pset 2. Cost is calculated by a FF ignorning any delete effects and negative preconditions. However, it currently works worse than the basic BFS since the queue doesn't distinguish well between the many actions that are the same "distance" from the goal. As a result it add superflous actions since they are queued sooner but still accomplish the overall goal. Improvements will either add depth limiting or will change cost function to be a cumulative cost of the path so far by adding up the various actions. Run time is approx 30s
    runs with: python -B -m activity_planner_best_first /home/amotts/Documents/MIT_FA23/padm-project/activity_planning_pddl_domain.pddl /home/amotts/Documents/MIT_FA23/padm-project/activity_planning_pddl_problem.pddl


## Part 2: Motion Planning

Did some file reorganization and upkeep. Removed the repository for the padm-project-2023f so the files could be added to my repository. Removed redunadant nestled directories to shorten paths and better allow importing from files and tools provided

motion_planner_scratch: Planning notes, observations, and performance comments about motion planning files and code

motion_planning_v1: Largely copied off of minimal_example.py. Implimented code for basic rrt including rrt nodes and rrt protocol for basic rrt. After testing, decided to include a percentage of samples to be the goal to speed things along. Included optional goal_function to allow for future flexibility in what is considered reaching the goal. Testing in 2D cartesian form showed over a 60% reduction in number of nodes explored to find goal. TO DO: a lot lol but implement the necessary function using the provided functions for the world

tests.py: file for creating test cases to test individual function performance:
    Contents:
    - 2D cartesian test for basic RRT