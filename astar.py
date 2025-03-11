import heapq
import numpy as np


def heuristic(start, goal):
    return abs(goal[0] - start[0]) + abs(goal[1] - start[1])



def astar_path(start, goal,obstacles):

    start,goal = tuple(np.round(start).astype(int)), tuple(np.round(goal).astype(int))


    open_set= []
    heapq.heappush(open_set,(0,start))


    closed_set= set()

    came_from ={}

    g_score = {start:0}
    f_score = {start:heuristic(start,goal)}

    while open_set:
        _,current=heapq.heappop(open_set)

        if current == goal:
            path=[]

            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        closed_set.add(current)


        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            neighbor = (current[0] + dx, current[1] + dy)

            if any(np.linalg.norm(np.array(neighbor)-np.array(ob["center"])) < ob["radius"]for ob in obstacles):
                continue

            tentative_g_score = g_score[current] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor,goal)
                heapq.heappush(open_set,(f_score[neighbor],neighbor))

    return []