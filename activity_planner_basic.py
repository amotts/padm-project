## Create a BFS planner based on the example planner in the pddl_parser library and the Pset 1 informed search BFS implementation
# python -B -m activity_planner_basic pddl_domain.pddl pddl_problem.pddl


## Future plans - implement best first using Priority Queue w/ cost as the path length to satisfy goal if when we perform action, no negative effects
# Use Pset 2 Best First Search


from pddl_parser.PDDL import PDDL_Parser

class Planner:
    def solve(self, domain, problem):
        activity=PDDL_Parser()
        activity.parse_domain(domain)
        activity.parse_problem(problem)
        state = activity.state
        pos_goals = activity.positive_goals
        neg_goals = activity.negative_goals
        # Check for goal satisfaction from initial conditions
        if self.satisfied(state, pos_goals, neg_goals):
            return []

        ground_actions = []
        for action in activity.actions:
            for act in action.groundify(activity.objects, activity.types):
                ground_actions.append(act)
        
        ## impliment BFS on activity to find path
        visited = set()
        visited.add(state) #Visited list starts with initial state
        queue = [state]
        partial_paths = [None]

        while queue:
            state = queue.pop(0)
            path = partial_paths.pop(0)
            for action in ground_actions:
                if self.satisfied(state, action.positive_preconditions, action.negative_preconditions):
                    new_state = self.perform(state, action) 
                    if new_state not in visited:
                        if self.satisfied(new_state, pos_goals, neg_goals):
                            complete_path = [action]
                            while path:
                                act, path = path
                                complete_path.insert(0, act)
                            return complete_path
                        visited.add(new_state)
                        queue.append(new_state)
                        partial_paths.append((action, path))
        return(None)



    def satisfied(self, state, pos_conditions, neg_conditions):
        return pos_conditions.issubset(state) and neg_conditions.isdisjoint(state)
    
    def perform(self, state, action):
        return state.difference(action.del_effects).union(action.add_effects)

#-----------------------------------------------
# Main
# -----------------------------------------------
if __name__ == '__main__':
    import sys, time
    start_time = time.time()
    domain = sys.argv[1]
    problem = sys.argv[2]
    verbose = len(sys.argv) > 3 and sys.argv[3] == '-v'
    planner = Planner()
    plan = planner.solve(domain, problem)
    print('Time: ' + str(time.time() - start_time) + 's')
    if plan is not None:
        print('plan:')
        for act in plan:
            print(act if verbose else act.name + ' ' + ' '.join(act.parameters))
    else:
        sys.exit('No plan was found')