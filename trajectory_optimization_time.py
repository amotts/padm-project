from pydrake.solvers import MathematicalProgram, Solve
import numpy as np
import matplotlib.pyplot as plt




def get_optimal_tragectory(start, end):
# Create an empty MathematicalProgram named prog (with no decision variables, 
# constraints or cost function)
    prog = MathematicalProgram()

    h = 100


    x = prog.NewContinuousVariables(7, h, "x")
    y = prog.NewContinuousVariables(7, h-1, "y")
    t = prog.NewContinuousVariables(h-1, "t")

    ## Initial Drawer 1.3468115429378031, -1.1855967232420714, -2.2134869146883633, -1.7960922256926946, -0.2526004518888567, 2.928824959281513, -0.42977342085737813
    prog.AddConstraint(x[0,0] == start[0])   
    prog.AddConstraint(x[1,0] == start[1])
    prog.AddConstraint(x[2,0] == start[2])
    prog.AddConstraint(x[3,0] == start[3])
    prog.AddConstraint(x[4,0] == start[4])
    prog.AddConstraint(x[5,0] == start[5])
    prog.AddConstraint(x[6,0] == start[6])

        
    ## Goal Stove 0.01064202542796632, 1.4, 0.018605815479346265, -0.5, -0.05907068003938907, 3.1, 0.7906325747027143
    prog.AddConstraint(x[0,h-1] == end[0])   
    prog.AddConstraint(x[1,h-1] == end[1])
    prog.AddConstraint(x[2,h-1] == end[2])
    prog.AddConstraint(x[3,h-1] == end[3])
    prog.AddConstraint(x[4,h-1] == end[4])
    prog.AddConstraint(x[5,h-1] == end[5])
    prog.AddConstraint(x[6,h-1] == end[6])

    for i in range(7):
        prog.AddConstraint(y[i,0]==0)
        prog.AddConstraint(y[i, h-2] == 0)

    prog.AddBoundingBoxConstraint(-0.4, 0.4, y)
    prog.AddBoundingBoxConstraint(0.0001, 0.1, t)


    for i in range(h-1):
        for j in range(7):
            prog.AddConstraint(x[j,i] + t[i]*y[j,i]==x[j,i+1])



    prog.AddCost(sum(t[i] for i in range(h-1)))




    # print(prog)

    # Now solve the optimization problem.
    result = Solve(prog)

    # print out the result.
    print("Success? ", result.is_success())
    # Print the solution to the decision variables.
    print('x* = ', result.GetSolution(x))
    print('y* = ', result.GetSolution(y))
    print('t* = ', result.GetSolution(t))
    # Print the optimal cost.
    print('optimal cost = ', result.get_optimal_cost())
    # Print the name of the solver that was called.
    print('solver is: ', result.get_solver_id().name())



    steps_raw = result.GetSolution(x)
    times = result.GetSolution(t)
    print(steps_raw)
    steps = list()
    for i in range(h):
        steps.append(list())
    for angle in steps_raw:
        for i in range(len(angle)):
            steps[i].append(angle[i])
    print(steps)

    return(steps,times)


def main():
    get_optimal_tragectory()

if __name__ == '__main__':
    main()
