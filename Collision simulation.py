import numpy as np
import heapq
import matplotlib.pyplot as plt
import ast
import math
import pandas as pd
def parse_tuple(string):
    try:
        s = ast.literal_eval(str(string))
        if type(s) == tuple:
            return s
        return
    except:
        return

def convert_pathes_to_routes(df_pathes):
    optimal_routes =[]
    #Iterate over each column
    for col_idx in range(df_pathes.shape[1]):
        route_list = list()
        for row_idx in range(df_pathes.shape[0]):
            if isinstance(parse_tuple(df_pathes.values[row_idx,col_idx]),tuple):
                route_list.append(parse_tuple(df_pathes.values[row_idx,col_idx]))
        optimal_routes.append(route_list)
    return optimal_routes


route = []
new_route = []
k = 2

# load the map file into numpy array
with open('map_data.txt', 'rt') as infile:
    grid1 = np.array([list(line.strip()) for line in infile.readlines()])
print('Grid shape', grid1.shape)

grid1[grid1 == '@'] = 1  # object on the map
grid1[grid1 == 'T'] = 1  # object on the map
grid1[grid1 == '.'] = 0  # free on map

grid = np.array(grid1.astype(np.int))

# Plot the map
fig, ax = plt.subplots(figsize=(10, 5))
ax.imshow(grid, cmap=plt.cm.ocean)



#route = [[((10, 10), 0), ((10, 11), 1), ((10, 12), 2), ((10, 13), 3), ((10, 14), 4), ((10, 15), 5), ((10, 16), 6), ((10, 17), 7)],
#         [((11, 10), 0), ((11, 11), 1), ((11, 12), 2), ((11, 13), 3), ((11, 14), 4), ((11, 15), 5), ((11, 16), 6), ((11, 17), 7)],
#         [((12, 10), 0), ((12, 11), 1), ((12, 12), 2), ((12, 13), 3), ((12, 14), 4), ((12, 15), 5), ((12, 16), 6), ((12, 17), 7)]]

df_optimal_pathes = pd.read_csv("Results.csv")
route = convert_pathes_to_routes(df_optimal_pathes)


##############################################################################
# heuristic function for path scoring
##############################################################################

def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2


##############################################################################
# path finding function
##############################################################################

def astar(array, start, goal, y):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    print("Re-calculation A-star algorithm")
    start_node = (start)

    close_set = set()
    came_from = {start_node: 0}
    gscore = {start_node: 0}
    fscore = {start_node: heuristic(start[0], goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start_node], start_node))
    is_collision = 0

    while oheap:

        current = heapq.heappop(oheap)[1]

        if current[0] == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = list(reversed(data))
            return data

        close_set.add(current)

        # Add stay at current position option
        neighbor = ((current[0][0], current[0][1]), current[1] + 1)
        came_from[neighbor] = (current)
        gscore[neighbor] = gscore[current] + 1
        fscore[neighbor] = gscore[current] + 1 + heuristic(neighbor[0], goal)
        heapq.heappush(oheap, (fscore[neighbor], neighbor))

        for i, j in neighbors:

            neighbor = ((current[0][0] + i, current[0][1] + j), current[1] + 1)

            tentative_g_score = gscore[current] + heuristic(current[0], neighbor[0])

            ##Check for collision
            for a in range(0, y):
                if len(route[a]) > neighbor[1]:
                    if neighbor[0] == route[a][neighbor[1]][0]:
                        is_collision = 1
                        break
            if is_collision == 1:
                is_collision = 0
                continue

            if 0 <= neighbor[0][0] < array.shape[0]:
                if 0 <= neighbor[0][1] < array.shape[1]:
                    if array[neighbor[0][0]][neighbor[0][1]] == 1:
                        # array bound 1 on map
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor[0], 0):
                continue

            if tentative_g_score < gscore.get(neighbor[0], 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = (current)
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor[0], goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False


def search_for_interrupt_plan():
    actions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    open = list()
    pos = list()
    step = 0
    global k

    #Set starting positions for all agents into a node
    for x in range(0, len(route)):
        pos.append(route[x][0][0])

    start = (pos, k, step)
    open.append(start)
    goals = list()

    while (len(open)>0):
        node = open.pop()

        step = node[2]+1

        for action in actions:
            new_node = apply(action, node, step)
            if (is_goal(new_node)):
                goals.append(new_node)
            else:
                open.append(new_node)

def is_goal(self):
    global k
    if k == 0:
        return True
    for x in range(1, len(self[0])):
        if self[0][x] is not route[x][-1][0]:
            return False
    return True
    # TODO: IS K=0 OR ALL AT AGOAL


def apply(action, node, step):
    global k
    tmp_node = list()
    pos = list()

    if step < len(route[0]):
        #If action of Agent 0 is according the plan - don't decrease k
        if route[0][step][0] == (node[0][0][0] + action[0], node[0][0][1] + action[1]):
            for x in range(0, len(route)):
                print(str(x)+" "+ str(step)+" ")
                pos.append(route[x][step][0])
            tmp_node = (pos, k, step)
            return tmp_node
        else:
            tmp_node = calc_new_routes(action, node, step)
    else:
        tmp_node = calc_new_routes(action, node, step)
    return tmp_node

def calc_new_routes(action, node, step):
    global k
    tmp_node = list()
    pos = list()
    tmp_node.append((node[0][0][0] + action[0], node[0][0][1] + action[1]))
    agent_that_interrupt = 0

    #check interrupt for all agent with agent0
    for x in range(1, len(route)):

        #if more steps for agent x so check interrupt
        if step < len(route[x]) and step < len(route[0]):

            # If interrupt - update plan using A* for all agents
            if tmp_node[0] == route[x][step][0] and k > 0:
                agent_that_interrupt = x
                print("agent: "+str(agent_that_interrupt)+" interrupted. attacker moved: "+str(node)+" step "+ str(step))
                for a in range(0, len(route)-1):
                    if a == 0:
                        if step >= len(route[a]):
                            route[a].append((tmp_node[0], step))
                        else:
                            route[a][step] = (tmp_node[0], step)

                    # send agent0 to A* (grid, start = next new step, goal, agent no.)
                    if a is not agent_that_interrupt:
                        start = route[a][step]
                        goal = route[a][-1][0]
                        if step >= 1:
                            new_route = []
                            for h in range(0, step):
                                new_route.append(route[0][h])
                            route[a] = astar(grid, start, goal, a)
                            for h in route[0]:
                                new_node = ((h[0]), new_route[-1][1] + 1)
                                new_route.append(new_node)
                            route[a] = new_route
                        else:
                            route[a] = astar(grid, start, goal, a)
                    else:
                        new_route = []
                        for h in range(0, step):
                            new_route.append(route[a][h])
                        start = route[a][step - 1]
                        goal = route[a][-1][0]
                        if step >= 1:
                            route[a] = astar(grid, start, goal, a)
                            for h in route[a]:
                                new_node = ((h[0]), new_route[-1][1] + 1)
                                new_route.append(new_node)
                            route[a] = new_route
                        else:
                            route[a] = astar(grid, start, goal, a)
                k -= 1

    # Set starting positions for all agents into a node
    pos.append(tmp_node[0])
    for x in range(1, len(route)):
        if step < len(route[x]):
            pos.append(route[x][step][0])
        else:
            pos.append(route[x][-1][0])
    tmp_node = (pos, k, step)
    return tmp_node


search_for_interrupt_plan()
print(route)

for x in range(0, len(route)):

    start = route[x][0][0]
    goal = route[x][-1][0]

    if route:
        print('-'*20, 'Agent No. - ', x, '-'*20)
        print('Start point - ', start, '\nGoal point  - ', goal)
        print('Len of the route - ', len(route[x]))
        print('-'*56, '\n')

    ##############################################################################
    # plot the path
    ##############################################################################

    # extract x and y coordinates from route list
    x_coords = []
    y_coords = []

    if route:
        for i in (range(0, len(route[x]))):
            x1 = route[x][i][0][0]
            y1 = route[x][i][0][1]

            x_coords.append(x1)
            y_coords.append(y1)

    # plot path
    ax.scatter(start[1], start[0], marker="*", color="red", s=50)
    ax.scatter(goal[1], goal[0], marker="*", color="purple", s=50)
    ax.plot(y_coords, x_coords, color="blue")

plt.show()




