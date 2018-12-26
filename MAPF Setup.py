# -*- coding: utf-8 -*-
"""
Created on Sat Dec 22 08:05:00 2018

@author: rotem
"""
#############################################################################
# import packages
##############################################################################

import random
import numpy as np
import heapq
import matplotlib.pyplot as plt
import pandas as pd


route = []

# load the map file into numpy array
with open('map_data.txt', 'rt') as infile:
    grid1 = np.array([list(line.strip()) for line in infile.readlines()])
print('Grid shape', grid1.shape)

grid1[grid1 == '@'] = 1 #object on the map
grid1[grid1 == 'T'] = 1 #object on the map
grid1[grid1 == '.'] = 0 #free on map

grid = np.array(grid1.astype(np.int))

# Get agent number
num_of_agents = int(input('Number of Agents: '))

# Plot the map
fig, ax = plt.subplots(figsize=(10, 5))
ax.imshow(grid, cmap=plt.cm.ocean)

##############################################################################
# heuristic function for path scoring
##############################################################################

def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2


##############################################################################
# get start and goal for all agents (not the lead)
##############################################################################

def get_positions (grid, lead_start, lead_goal):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    neighbor = random.choice(neighbors)

    new_start = lead_start[0] + neighbor[0], lead_start[1] + neighbor[1]
    new_goal = lead_goal[0] + neighbor[0], lead_goal[1] + neighbor[1]
    np.array(grid)

    return new_start, new_goal


##############################################################################
# path finding function
##############################################################################

def astar(array, start, goal):
    neighbors = [(0, 0), (0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    start_node = (start, 0)

    close_set = set()
    came_from = {start_node: 0}
    gscore = {start_node: 0}
    fscore = {start_node: heuristic(start, goal)}
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
        for i, j in neighbors:

            neighbor = ((current[0][0] + i, current[0][1] + j), current[1]+1)

            tentative_g_score = gscore[current] + heuristic(current[0], neighbor[0])

            ##Check for collision
            for a in range(0, x):
                if len(route[a]) > neighbor[1]:
                    if neighbor[0] == route[a][neighbor[1]][0]:
                        #print('Agent no. - ', a, 'Step no. - ', current[1], 'Potential Collision between - ', neighbor[0], ' and ', route[a][neighbor[1]-1][0])
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

##############################################################################
# Get random start & goal for the leader
##############################################################################
secure_random = random.SystemRandom()
lead_start = (random.choice(np.argwhere(np.array(grid) == 0)))
lead_goal = (random.choice(np.argwhere(np.array(grid) == 0)))

for x in range(0, num_of_agents):

    a_start, a_goal = get_positions(grid, lead_start, lead_goal)

    start = (a_start[0], a_start[1])
    goal = (a_goal[0], a_goal[1])

    ##############################################################################
    # Calling A*
    ##############################################################################
    route.append(astar(grid, start, goal))

    if route:
        print('-'*20, 'Agent No. - ', x, '-'*20)
        print('Start point - ', a_start, '\nGoal point  - ', a_goal)
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

            grid[x1][y1] = i

            x_coords.append(x1)
            y_coords.append(y1)

    # plot path
    ax.scatter(start[1], start[0], marker="*", color="red", s=50)
    ax.scatter(goal[1], goal[0], marker="*", color="purple", s=50)
    ax.plot(y_coords, x_coords, color="blue")

#Write to Excel
df_res = pd.DataFrame(route)
df_res = df_res.T
writer_xl = pd.ExcelWriter('Results.xlsx', engine='openpyxl')
df_res.to_excel(writer_xl, sheet_name='Agents', index=False)
writer_xl.save()
writer_xl.close()

plt.show()