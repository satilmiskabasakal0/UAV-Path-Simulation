import numpy as np
from astar import astar_path
import config


AVAILABLE_TRACKING_MODES= config.TRACKING_MODES


def direct_follow(follower, dt, leader):
    direction = leader.pos - follower.pos
    if np.linalg.norm(direction) > 0:
        direction = direction / np.linalg.norm(direction)
        follower.pos += direction * follower.speed * dt

def predictive_follow(follower, dt, leader):

    predicted_leader_pos = leader.predict_future_position(dt * 5)

    direction = predicted_leader_pos - follower.pos
    if np.linalg.norm(direction) > 1.0:
        direction = direction / np.linalg.norm(direction)
        follower.pos += direction * follower.speed * dt




def astar_follow(follower, dt, leader_pos, obstacles):



    if len(follower.path) == 0 or follower.replan_flag:
        follower.path = astar_path(follower.pos, leader_pos, obstacles)
        follower.replan_flag = False


    if len(follower.path) > 0:
        target = np.array(follower.path[0])
        direction = target - follower.pos

        if np.linalg.norm(direction) < 1.0:
            follower.path.pop(0)
        else:
            direction = direction / np.linalg.norm(direction)
            follower.pos += direction * follower.speed * dt




