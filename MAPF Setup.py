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
with open('room1.txt', 'rt') as infile:
    grid1 = np.array([list(line.strip()) for line in infile.readlines()])
print('Grid shape', grid1.shape)

grid1[grid1 == '@'] = 1 #object on the map
grid1[grid1 == 'T'] = 1 #object on the map
grid1[grid1 == '.'] = 0 #free on map

grid = np.array(grid1.astype(np.int))

# Get agent number
num_of_agents = int(input('Number of Agents: '))
num_of_routes = int(input('Number of Routes: '))

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

def get_positions(agent_number):

    starts = [(14, 16), (14, 17), (14, 18), (15, 16), (15, 17), (15, 18), (15, 19), (16, 17), (16, 18), (16, 19)]
    goals = [(34, 14), (34, 15), (35, 14), (35, 15), (35, 16), (36, 14), (36, 15), (36, 16), (37, 15), (37, 16)]

    new_start = starts[agent_number]
    new_goal = goals[agent_number]

    return new_start, new_goal


##############################################################################
# path finding function
##############################################################################

def astar(array, start, goal):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

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

        #Add stay at current position option
        #neighbor = ((current[0][0], current[0][1]), current[1] + 1)
        #came_from[neighbor] = (current)
        #gscore[neighbor] = gscore[current]+1
        #fscore[neighbor] = gscore[current]+1 + heuristic(neighbor[0], goal)
        #heapq.heappush(oheap, (fscore[neighbor], neighbor))

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


for no_of_per in range(0, num_of_routes):

    route = []
    t = [0,1,2,3,4,5,6,7,8,9]
    current_agent = []
    for i in range(1):
         random.shuffle(t)
         current_agent.append(list(t))


    for x in range(0, num_of_agents):

        a_start, a_goal = get_positions(current_agent[0][x])

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

    # Write to Excel
    df_res = pd.DataFrame(route)

    file_name = []
    file_name.append('Results')
    file_name.append(str(no_of_per))
    file_name.append('.xlsx')
    file_n = ' '.join(file_name)
    file_n.replace(" ", "")

    writer_xl = pd.ExcelWriter(file_n, engine='openpyxl')
    df_res.to_excel(writer_xl, sheet_name='Agents', index=False)
    writer_xl.save()
    writer_xl.close()



