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



def get_action_plan(planner, domain, problem):
    cmd = ['python', '-B', '-m', planner, domain, problem]
    result = subprocess.run(cmd, stdout=subprocess.PIPE).stdout.decode('utf-8')
    print(result)

    plan = result.splitlines()
    plan.pop(0)
    plan.pop(0)
    for i in range(len(plan)):
        plan[i] = plan[i].split(" ")
        while len(plan[i]) < 5:
            plan[i].append(None)
    return(plan)

def main():
    cmd = ['python', '-B', '-m', 'activity_planner_basic', 'pddl_domain.pddl', 'pddl_problem_simple.pddl']
    result = subprocess.run(cmd, stdout=subprocess.PIPE).stdout.decode('utf-8')
    print(result, type(result))

    plan = result.splitlines()
    plan.pop(0)
    plan.pop(0)
    for i in range(len(plan)):
        plan[i] = plan[i].split(" ")
    print(plan)

    
if __name__ == '__main__':
    main()