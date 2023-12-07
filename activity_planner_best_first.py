## Create a BFS planner based on the example planner in the pddl_parser library and the Pset 1 informed search BFS implementation
# python -B -m activity_planner_best_first activity_planning_pddl_domain.pddl activity_planning_pddl_problem.pddl


## Future plans - implement best first using Priority Queue w/ cost as the path length to satisfy goal if when we perform action, no negative effects
# Use Pset 2 Best First Search

##
""" 
Time: 28.063291549682617s
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


Works, however will need to implement a dept limiter to prioritize cost amongst those on same level. Currently appears to be choosing cheapest option however 
since so many options are the same, adds superflous steps. Could either use a cumulative cost function or some sort of depth limiter.
UPDATE cost+prev_cost did not work. Stopped after several minutes with total costs in the mid 20s.Too computationally intense

"""
## 

from pddl_parser.PDDL import PDDL_Parser
from queue import PriorityQueue

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
        queue = PriorityQueue()
        queue.put((0,[state, None]))


        while queue:
            prev_cost, queue_head = queue.get()
            state = queue_head[0]
            path = queue_head[1]
            for action in ground_actions:
                if self.satisfied(state, action.positive_preconditions, action.negative_preconditions):
                    new_state = self.perform(state, action)
                    cost = self.get_cost(activity, new_state) 
                    if new_state not in visited:
                        if self.satisfied(new_state, pos_goals, neg_goals):
                            complete_path = [action]
                            while path:
                                act, path = path
                                complete_path.insert(0, act)
                            return complete_path
                        visited.add(new_state)
                        queue.put((cost, [new_state, (action, path)]))
                        print(cost)
                        
        return(None)




    def get_cost(self, activity, state):
        pos_goals = activity.positive_goals
        neg_goals = activity.negative_goals
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
                if self.FF_satisfied(state, action.positive_preconditions, action.negative_preconditions):
                    new_state = self.FF_perform(state, action) 
                    if new_state not in visited:
                        if self.FF_satisfied(new_state, pos_goals, neg_goals):
                            complete_path = [action]
                            while path:
                                act, path = path
                                complete_path.insert(0, act)
                            return len(complete_path)
                        visited.add(new_state)
                        queue.append(new_state)
                        partial_paths.append((action, path))
        return(None)








    def satisfied(self, state, pos_conditions, neg_conditions):
        return pos_conditions.issubset(state) and neg_conditions.isdisjoint(state)
    
    def FF_satisfied(self, state, pos_conditions, neg_conditions):
        return pos_conditions.issubset(state)
    
    def perform(self, state, action):
        return state.difference(action.del_effects).union(action.add_effects)
    
    def FF_perform(self, state, action):
        return state.union(action.add_effects)

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