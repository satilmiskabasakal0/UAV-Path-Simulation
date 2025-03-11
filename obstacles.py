import numpy as np
import random
import config

GLOBAL_OBSTACLES = None


def create_obstacles():
    global GLOBAL_OBSTACLES
    if GLOBAL_OBSTACLES is None:

        attempts = 0
        obstacles_list = []

        while len(obstacles_list) < config.NUM_OBSTACLES and attempts < config.MAX_ATTEMPTS:
            center = np.array([
                random.uniform(config.MAX_RADIUS, abs(config.AREA_SIZE - config.MAX_RADIUS)),
                random.uniform(config.MAX_RADIUS, abs(config.AREA_SIZE - config.MAX_RADIUS))
            ])
            radius = random.uniform(config.MIN_RADIUS, config.MAX_RADIUS)

            overlap = False
            for obstacle in obstacles_list:
                if np.linalg.norm(center - obstacle["center"]) < (radius + obstacle["radius"]):
                    overlap = True
                    break

            if not overlap:
                obstacles_list.append({'center': center, 'radius': radius})

            attempts += 1

        if attempts == config.MAX_ATTEMPTS:
            print(
                f"Obstacle areas could not be fully defined in {config.MAX_ATTEMPTS} attempts. Only {len(obstacles_list)} obstacles were created.")

        GLOBAL_OBSTACLES = obstacles_list

    return GLOBAL_OBSTACLES


def is_inside_obstacle(pos, obstacles_list):
    for obstacle in obstacles_list:
        if np.linalg.norm(pos - obstacle["center"]) < obstacle["radius"]:
            return True
    return False


def get_exit_direction(pos, obstacles_list):
    best = None
    best_margin = None
    for obstacle in obstacles_list:
        center = obstacle["center"]
        radius = obstacle["radius"]
        vec = pos - center
        dist = np.linalg.norm(vec)
        if dist < radius:
            margin = radius - dist
            if best is None or margin < best_margin:
                best_margin = margin
                best = vec
    if best is None or np.linalg.norm(best) == 0:
        return None

    else:
        return best / np.linalg.norm(best)
