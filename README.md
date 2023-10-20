# PADM Final Project
Repository for all files relating to the final project for Principles of Autonomy and Design Fall 2023


## Part 1: Activity Planning 

activity_planning_scratch.txt: document containing scratchwork for determinining structure of PDDL enviornmentt to be created in new file

activity_planning_pddl_domain: establish a kitchen domain with locations and things. Locations include countertops, stovetops, drawers, cabinets. Options for actions are to open or close locations or move object from one location to another

activity_planning_pddl_problem: establish a problem within a kitchen domain with two stoves, 3 counters, 3 cabinets, and 3 drawers, and a gripper. All closeable locations are closed. Goal is occupy counter with sugar and red drawer with spam and have an empty gripper.

Notes: Currently, multiple actions are offered that would never be needed and are not representative of the real world (e.g. opening or closing a counter or stovetop). Since these predicates should have no bearing on any available actions and would never be on a shortest path to acchieve the goal, I don't expect them to pose any issues. They will increase computational load since they will always be an option to move the gripper to the location and change the state of the door, but I will deal with that issue if it comes up.

**Need to spell check and formatting check domain and problem**
