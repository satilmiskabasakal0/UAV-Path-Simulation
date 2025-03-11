import yaml

filename="config.yaml"



with open(filename, "r") as f:
        config= yaml.safe_load(f)



# Simulation Settings
AREA_SIZE = config["simulation"]["area_size"]
SIM_TIME = config["simulation"]["sim_time"]
DT = config["simulation"]["dt"]
NUM_STEPS = int(SIM_TIME / DT)
CHANGE_INTERVAL=config["simulation"]["change_interval_time"]

# Engel Ayarları
NUM_OBSTACLES = config["obstacles"]["num_obstacles"]
MIN_RADIUS = config["obstacles"]["min_radius"]
MAX_RADIUS = config["obstacles"]["max_radius"]
MAX_ATTEMPTS = config["obstacles"]["max_attempts"]


# Takip modları
TRACKING_MODES=config["tracking_modes"]["modes"]
