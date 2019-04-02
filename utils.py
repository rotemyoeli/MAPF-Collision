import numpy as np
import heapq
import logging


'''
Check if pos is a legal position in the map
'''
def is_legal(grid, pos):
    if 0 <= pos[0] < grid.shape[0]:
        if 0 <= pos[1] < grid.shape[1]:
            if grid[pos[0]][pos[1]] == 1:
                return False # array bound 1 on map
        else:
            return False # array bound y walls
    else:
        return False # array bound x walls
    return True

'''
Return a dictionary that maps every location to the distance from the given start
'''
def dijkstra(grid, source, operators):
    dist_to_source = dict()
    dist_to_source[source]=0

    oheap = []
    closed = set()
    heapq.heappush(oheap, (0, source))

    while oheap:
        (g, pos) = heapq.heappop(oheap)
        closed.add(pos)

        for i, j, cost in operators:
            neighbor = (pos[0] + i, pos[1] + j)
            new_g = g+cost

            if is_legal(grid, neighbor)==False:
                continue

            if neighbor in closed:
                continue

            if neighbor in dist_to_source:
                # If existing path is better
                if dist_to_source[neighbor]<=new_g:
                    continue
                # TODO: In the future,

            dist_to_source[neighbor]=new_g
            heapq.heappush(oheap,(new_g, neighbor))
    return dist_to_source


'''
    Reads a grid from a file and creates a numpy array object for it
'''
def read_grid(grid_file):
    with open(grid_file, 'rt') as infile:
        grid1 = np.array([list(line.strip()) for line in infile.readlines()])

    grid1[grid1 == '@'] = 1  # object on the map
    grid1[grid1 == 'T'] = 1  # object on the map
    grid1[grid1 == '.'] = 0  # free on map

    return np.array(grid1.astype(np.int))



'''
    Print a given MDR solution object
'''
def print_solution(solution):
    delimiter = "\t"
    num_of_agents = len(solution[0].pos)

    # Header
    record = "k" + delimiter
    for agent in range(num_of_agents):
        record = "%s%d%s" % (record,agent,delimiter)
    logging.info(record)

    for mdr_node in solution:
        record = "%s%s" % (mdr_node.k,delimiter)
        for agent in range(num_of_agents):
            record = "%s%s%s" % (record,mdr_node.pos[agent],delimiter)
        logging.info(record)