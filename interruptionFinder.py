import heapq
import math
import copy
import numpy as np
import pathFinder
import time

class Node:

    def __init__(self, pos, k, step, parent,route=None):
        self.pos = pos
        self.k = k
        self.step = step
        self.parent = parent
        self.route = route


class InterruptionFinder(object):

    '''
        Applies an action of the interrupting agent. This may cause other agents to replan.      
    '''
    def apply(self, action, node, step, route_):
        route = copy.deepcopy(route_)
        tmp_node = list()
        pos = list()

        if step < len(route[0]):
            #If action of Agent 0 is according the plan - don't decrease the no. of remained error
            if route[0][step][0] == (node.pos[0][0] + action[0], node.pos[0][1] + action[1]):
                pos.append(route[0][step][0])
                for w in range(1, len(route)):
                    if step < len(route[w]):
                        pos.append(route[w][step][0])
                    else:
                        pos.append(route[w][-1][0])
                tmp_node = Node(pos, node.k, step, node,route = route)
                return tmp_node, route

            #If action of Agent 0 is not according the plan - decrease the no. of remained error
            else:
                 tmp_node, route = self.calc_new_routes(action, node, step, route)
        else:
            return None,None
        #check for collision and if there is:
        last = tmp_node.pos[0]
        for x in range(1, len(tmp_node.pos)):

            if tmp_node.pos[x] == last and (step-1) < len(route[x]):
                tmp_node.pos[x] = route[x][step - 1][0]
                route[x].insert(step, (tmp_node.pos[x], step))
                for j in range(step+1, len(route[x])):
                    route[x][j] = (route[x][j][0],  route[x][j][1] + 1)

            #last = tmp_node.pos[x]  #Shimon: ask Rotem what this row good for

        return tmp_node, route

    def calc_new_routes(self, action, node, step, route_):
        route = copy.deepcopy(route_)
        tmp_node = list()
        pos = list()
        # set the no. of agent that making the move for interruptions
        a = 0

        tmp_node.append((node.pos[0][0] + action[0], node.pos[0][1] + action[1]))

        #If there are no more errors left then return...
        #TODO: Check if this code is can be executed because k already checked before
        if node.k == 0:
            for x in range(0, len(route)):
                if step < len(route[x]):
                    pos.append(route[x][step][0])
                else:
                    pos.append(route[x][-1][0])
            tmp_node = Node(pos, 0, step, node)

            return tmp_node


        # Send Agent-0 to A* (grid, start = next new step, goal, agent no.)
        # if more steps for agent x so check interrupt
        if step < len(route[0]):
            new_route = []
            for h in range(0, step):
                new_route.append(route[a][h])

            start = (((node.pos[0][0] + action[0], node.pos[0][1] + action[1])), step)
            goal = route[a][-1][0]

            if step >= 1:
                route[a] = pathFinder.astar(self.grid, start, goal, a, route)
                for h in route[a]:
                    new_node = ((h[0]), new_route[-1][1] + 1)
                    new_route.append(new_node)
                route[a] = new_route
            else:
                route[a] = pathFinder.astar(self.grid, start, goal, a, route)
        else:
            new_node = route[a][-1][0]
            route[a].append(new_node)

        # Set positions for all agents into a node
        pos.append(tmp_node[0])
        for x in range(1, len(route)):
            if step < len(route[x]):
                pos.append(route[x][step][0])
            else:
                pos.append(route[x][-1][0])
        tmp_node = Node(pos, node.k-1, step, node,route=route)

        return tmp_node, route

    def is_goal(self, node):

        for x in range(0, len(node.pos)):
            if node.pos[x] is not self.goal_pos[x] and node.pos[x] is not None:
                return False
        return True


    '''
    search_node contains position of all agent, allowed abnormal moves, current time step
    routes contains the planned routes for all agents [ including the path they passed so far ]
    '''
    def f_interrupt(self, search_node, routes):

        cost_without_interrupts = sum([len(route) for route in routes])

        # An overestimate of the cost added by the abnormal moves
        remaining_abnormal_moves = search_node.k
        num_of_agents = len(search_node.pos)

        # The idea is that the maximal addition for each abnormal move is that it will delay all agents by one time step.
        return cost_without_interrupts + num_of_agents * remaining_abnormal_moves



    '''
     route = list containing the planned route for each agent 
    '''
    def search_for_interrupt_plan(self, grid, k, original_routes, timeout):
        self.k = k
        self.grid = grid
        start_time = time.time()
        total_expanded_nodes = 0
        actions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1), (0, 0)]
        # actions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        open = []
        step = 0

        num_of_agents = len(original_routes)

        # Set starting positions for all agents into a node
        self.start_pos = [original_routes[x][0][0] for x in range(0, num_of_agents)]

        # Compute the goal positions
        self.goal_pos = [original_routes[x][-1][0] for x in range(0, num_of_agents)]

        # pos = a list that contains location of all agents
        # k = number of allowed abnormal moves
        # step = current time step
        # None = the parent pointer

        start = Node(pos=self.start_pos, k=k, step=step, parent=None, route=original_routes)

        f_score = self.f_interrupt(start, original_routes)
        fscore = {start: f_score}
        heapq.heappush(open, (fscore[start], start))

        goals = list()
        best_goal_value = 0
        while (len(open) > 0):
            # Check for timeout
            if (time.time() - start_time) > timeout:
                return (0, original_routes, total_expanded_nodes, 1)

            node = heapq.heappop(open)[1]
            # count total expanded nodes
            total_expanded_nodes = total_expanded_nodes + 1
            step = node.step + 1

            # No need to expand a node that cannot improve the best goal
            if self.f_interrupt(node, node.route) <= best_goal_value:
                continue
            if self.is_goal(node):
                continue
            # print("%d" % (len(goals)))
            for action in actions:

                # calc the vector between start and goal
                anglediff = self.compute_anglediff_to_goal(action, node, original_routes)

                # print(anglediff, next_step_degrees, goal_degrees)
                if (anglediff >= 90 or anglediff <= -90):
                    continue

                if node.k == 0:
                    # no more err to use
                    pos = list()
                    for x in range(0, len(node.route)):
                        if step < len(node.route[x]):
                            pos.append(node.route[x][step][0])
                        else:
                            # TODO: Try to add none instead the last node and verify the code
                            pos.append(node.route[x][-1][0])
                    new_node = Node(pos, 0, step, node, node.route)
                    if (self.is_goal(new_node, new_node.route)):
                        goals.append(new_node)

                        # find the total cost of the current route
                        cost_new_node = 0
                        for m in range(0, len(new_node.route)):
                            cost_new_node = cost_new_node + new_node.route[m][-1][1]
                        if cost_new_node > best_goal_value:
                            best_goal_value = cost_new_node
                    else:
                        f_score = self.f_interrupt(new_node, original_routes)
                        fscore[new_node] = f_score + node.step + (np.random.random() / 1)
                        heapq.heappush(open, (fscore[new_node], new_node))

                    break

                # if action is legal (not collide in wall)
                if grid[node.pos[0][0] + action[0]][node.pos[0][1] + action[1]] == 1:
                    # array bound 1 on map
                    continue
                # if action of agent 0 is collide in other agent location, ignore action
                collision_with_exist_agent = self.is_colliding(action, node)
                if collision_with_exist_agent:
                    continue
                new_node, _ = self.apply(action, node, step, node.route)
                if new_node == None: # TODO: When does this occur?
                    continue
                if (self.is_goal(new_node, new_node.route)):
                    goals.append(new_node)

                    # find the total cost of the current route
                    cost_new_node = 0
                    for m in range(0, len(new_node.route)):
                        cost_new_node = cost_new_node + new_node.route[m][-1][1]
                    if cost_new_node > best_goal_value:
                        best_goal_value = cost_new_node
                else:
                    f_score = self.f_interrupt(new_node, new_node.route)
                    fscore[new_node] = f_score + node.step + (np.random.random() / 1)
                    heapq.heappush(open, (fscore[new_node], new_node))

        # After all goals have been found
        max_step = 0
        best_goal = None
        for goal in goals:
            if goal.step > max_step:
                max_step = goal.step
                best_goal = goal

        # Reconstruct the solution
        solution = [best_goal]
        parent = best_goal.parent

        while parent is not None:
            solution.append(parent)
            parent = parent.parent

        # solution = list(reversed(solution))
        return(solution, original_routes, total_expanded_nodes, 0)


    '''
        Checks if performing the action in the given node will create a collision with another agent
        if all agents' follow their current routes.
    '''
    def is_colliding(self, action, node):
        collision_with_exist_agent = False
        for agent in range(1, len(node.route)):
            if node.pos[0][0] + action[0] == node.pos[agent][0] and node.pos[0][1] + action[1] == node.pos[agent][
                1]:
                collision_with_exist_agent = True
                break
        return collision_with_exist_agent

    '''
    Computes the different between the angle to the goal and the angle taken by the proposed action.
    '''
    def compute_anglediff_to_goal(self, action, node, route):
        goal_radians = math.atan2(route[0][-1][0][1] - node.pos[0][1], route[0][-1][0][0] - node.pos[0][0])
        goal_degrees = math.degrees(goal_radians)
        next_step_radians = math.atan2((node.pos[0][1] + action[1]) - node.pos[0][1],
                                       (node.pos[0][0] + action[0]) - node.pos[0][0])
        next_step_degrees = math.degrees(next_step_radians)
        anglediff = (next_step_degrees - goal_degrees + 180 + 360) % 360 - 180
        return anglediff
