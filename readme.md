# Multi-Algorithm UAV Path Planning and Simulation

This project demonstrates a **dynamic UAV (Unmanned Aerial Vehicle) simulation** where a leader UAV moves in a 2D environment with circular obstacles, and multiple followers track the leader using different path planning or tracking algorithms (A*, Predictive, Direct).


## Installation

1. **Clone or Download** the repository:
   ```bash
   git clone https://github.com/satilmiskabasakal0/UAV-Path-Simulation.git
   cd UAV-Path-Simulation

# Usage

## **Install Dependencies**:

   pip install -r requirements.txt
## Configure Simulation

Open config.yaml to change any parameters (like area_size, sim_time, dt, etc.).

## Run Simulation

Simply run:
bash

python main.py

This launches a Matplotlib animation window to display the leader and multiple followers in action.
At the end (or while closing), it attempts to save an animation in animations/ as animation_x.mp4.

## Add or Modify Algorithms

tracking_algorithms.py contains the logic (A*, Predictive, Direct). Implement your own or tweak existing ones.
Adjust followers in main.py (e.g., different speeds, different modes).

## Key Features

1. **Dynamic Configuration via `config.yaml`:**
    - The simulation parameters (e.g., `area_size`, `simulation_time`, `dt`, number of obstacles, etc.) can be easily changed in [`config.yaml`](./config.yaml).
    - No code modification is needed to change the simulation scale or obstacle counts—just update the YAML file.

2. **UAV Definitions in `uav.py`:**
    - **Leader (Leader class):**
        - Moves randomly, with obstacle avoidance.
        - Changes direction at configurable intervals (see `change_interval_time` in `config.yaml`).
        - If inside an obstacle, it finds an exit direction and moves out.
    - **Follower (Follower class):**
        - Follows the leader using various algorithms.
            - **A*** (A-Star) path planning
            - **Predictive** (estimates leader’s future position)
            - **Direct** (moves directly toward leader’s current position)
        - Avoids obstacles in the same manner as the leader.

3. **Obstacle Generation via `obstacles.py`:**
    - Creates circular obstacles placed randomly, with radii also randomized.
    - Ensures minimal overlap between obstacles.
    - Checks whether a UAV is inside an obstacle and provides a direction to exit.

4. **Dynamic Path Planning:**
    - For each time step, the leader updates its position.
    - Each follower picks the relevant tracking algorithm and updates accordingly.
    - Follower path or direction is re-planned as needed.

5. **Visualization and Animation:**
    - Uses **Matplotlib** to display:
        - The simulation area,
        - Obstacles (in red),
        - Leader’s path and marker,
        - Each Follower’s path and marker (in different colors).
    - Saves the animation to **MP4** if FFmpeg is installed.

6. **Easily Extendable:**
    - Add new algorithms in `tracking_algorithms.py`.
    - Add or remove followers in `main.py`.
    - Adjust speeds, intervals, obstacle counts, and more in `config.yaml`.

## Project Structure

```bash
UAV-Path-Simulation/
├── main.py                # Main entry point for running the simulation and animation
├── uav.py                 # UAV classes (Leader, Follower) - obstacle avoidance, updates, etc.
├── obstacles.py           # Creating obstacles, detection (is_inside_obstacle), exit direction
├── tracking_algorithms.py # Implementation of 'astar_follow', 'predictive_follow', 'direct_follow'
├── astar.py               # A* path planning methods (if needed)
├── config.py              # Loads and parses config.yaml for easy simulation changes
├── config.yaml            # YAML file controlling area size, sim time, dt, obstacle count/radii
├── requirements.txt       # Python dependencies
└── animations/            # MP4 animation files saved here




