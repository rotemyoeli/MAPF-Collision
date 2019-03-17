import heapq

##############################################################################
# heuristic function for A* path scoring
##############################################################################

def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2


##############################################################################
# path finding function
##############################################################################

def astar(array, start, goal, y, route):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    #neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
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
