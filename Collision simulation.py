import numpy as np
import heapq
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import csv
import ast
import sys

route = []
new_route = []
tmp_route = []
k = 1

org_total_g = 0
new_total_g = 0

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

# csv file name
filename = "results2.csv"

# reading csv file
with open(filename, 'rt') as csvfile:
    # creating a csv reader object
    csvreader = csv.reader(csvfile)
    # extracting each data row one by one
    for row in csvreader:
        tmp_route = []
        for y in range(0, len(row)):
            if row[y]:
                tmp_route.append(ast.literal_eval(row[y]))
        route.append(tmp_route)

for x in range(0, len(route)):
    org_total_g = org_total_g + len(route[x])
##############################################################################
# heuristic function for path scoring
##############################################################################

def heuristic(a, b):
    #if a[0] is not b[0] and a[1] is not b[1]:
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
    #else:
        #return 0

##############################################################################
# path finding function
##############################################################################

def astar(array, start, goal, y):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

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
            for a in range(0, y): #[k for k in range(0, len(route)) if k != y]:
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

    #Set starting positions for all agents into a node
    for x in range(0, len(route)):
        pos.append(route[x][0][0])

    start = (pos, k, step, None)
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

    # After all goals have been found
    max_step = 0
    best_goal = None
    for goal in goals:
        if goal[2] > max_step:
            max_step = goal[2]
            best_goal = goal

    # Reconstruct the solution
    solution = [best_goal]
    parent = best_goal[3]
    while parent is not None:
        solution.append(parent)
        parent = parent[3]
        solution = list(reversed(solution))
    return (solution)
    # Return the best_goal, it is the one that maximizes the makespan

def is_goal(self):
    if self[1] == 0:
        return True
    for x in range(1, len(self[0])):
        if self[0][x] is not route[x][-1][0]:
            return False
    return True

def apply(action, node, step):
    tmp_node = list()
    pos = list()

    if step < len(route[0]):
        #If action of Agent 0 is according the plan - don't decrease the no. of remained error
        if route[0][step][0] == (node[0][0][0] + action[0], node[0][0][1] + action[1]):
            pos.append(route[0][step][0])
            for w in range(1, len(route)):
                if step < len(route[w]):
                    pos.append(route[w][step][0])
                else:
                    pos.append(route[w][-1][0])
            tmp_node = (pos, k, step, node)
            return tmp_node

        #If action of Agent 0 is not according the plan - decrease the no. of remained error
        else:
            tmp_node = calc_new_routes(action, node, step)

    #check for collision and if there is:

    # stay at position

    return tmp_node

def calc_new_routes(action, node, step):
    tmp_node = list()
    pos = list()
    # set the no. of agent that making the move for interruptions
    a = 0

    tmp_node.append((node[0][0][0] + action[0], node[0][0][1] + action[1]))

    #If there are no more errors left then return...
    if node[1] == 0:
        for x in range(0, len(route)):
            if step < len(route[x]):
                pos.append(route[x][step][0])
            else:
                pos.append(route[x][-1][0])
        tmp_node = (pos, 0, step, node)

        return tmp_node


    # Send Agent-0 to A* (grid, start = next new step, goal, agent no.)
    # if more steps for agent x so check interrupt
    if step < len(route[0]):
        new_route = []
        for h in range(0, step):
            new_route.append(route[a][h])
        start = (((node[0][0][0] + action[0], node[0][0][1] + action[1])), step)
        goal = route[a][-1][0]
        if step >= 1:
            route[a] = astar(grid, start, goal, a)
            for h in route[a]:
                new_node = ((h[0]), new_route[-1][1] + 1)
                new_route.append(new_node)
            route[a] = new_route
        else:
            route[a] = astar(grid, start, goal, a)
    else:
        new_node = route[a][-1][0]
        route[a].append(new_node)
    #Run over all agents but agent 0
    for x in range(1, len(route)):

        #if there are more steps for agent x so check interrupt
        if step < len(route[x]) and step < len(route[0]):

            # !!!THIS LINE IS CHECKING THE INTERRUPTION!!!
            if route[a][step][0] == route[x][step][0] and node[1] > 0:

                #Insert stay action for the interrupt agent
                route[x].insert(step, (route[x][step-1][0], step))
                #Update steps no. for agent x
                for y in range(step+1, len(route[x])):
                    tmp_node2 = (route[x][y][0], route[x][y][1] + 1)
                    route[x][y] = tmp_node2

    # Set positions for all agents into a node
    pos.append(tmp_node[0])
    for x in range(1, len(route)):
        if step < len(route[x]):
            pos.append(route[x][step][0])
        else:
            pos.append(route[x][-1][0])
    tmp_node = (pos, node[1]-1, step, node)

    return tmp_node

solution = []
solution = search_for_interrupt_plan()
print(solution)

for x in range(0, len(solution)):
    new_total_g = new_total_g + len(solution[x])
    start = solution[x][0][0]
    goal = solution[x][-1][0]
    colors = ['r', 'black', 'black', 'black', 'black']#'purple', 'b', 'c', 'm', 'y', 'k', 'w']
    #if route:
        #print('-'*20, 'Agent No. - ', x, '-'*20)
        #print('Start point - ', start, '\nGoal point  - ', goal)
        #print('Len of the route - ', len(route[x]))
        #print('-'*56, '\n')

    colors = ['r', 'purple', 'b', 'c', 'm', 'y', 'k', 'w']
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
    ax.scatter(start[1], start[0], marker="*", color="black", s=50)
    ax.scatter(goal[1], goal[0], marker="*", color="y", s=50)
    if x <= len(colors):
        ax.plot(y_coords, x_coords, color=colors[x], label=x)
    else:
        ax.plot(y_coords, x_coords, color=colors[len(colors)], label=x)

    print(route[x])

print('Original g_score is - ', org_total_g)
print('New g_score is - ', new_total_g)

minor_ticks = np.arange(0, 261, 1)
ax.set_xticks(minor_ticks, minor=True)
ax.set_yticks(minor_ticks, minor=True)
ax.grid(which='both')
plt.show()
