import numpy as np
import config
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from uav import Leader,Follower
import obstacles
import tracking_algorithms
from obstacles import create_obstacles
import os
import matplotlib
matplotlib.use('TkAgg')

obstacles_list = create_obstacles()

tracking_modes = config.TRACKING_MODES


leader = Leader(obstacles_list=obstacles_list)

astar = Follower(obstacles_list=obstacles_list,tracking_modes=["astar"])
predictive = Follower(obstacles_list=obstacles_list,tracking_modes=["predictive"])
direct = Follower(obstacles_list=obstacles_list,tracking_modes=["direct"])

followers = [astar, predictive, direct]

followers_label = ["A*","Predictive","Direct"]
followers_colors = ["y","m","b"]

fig,ax = plt.subplots(figsize=(10,10))

ax.set_xlim(0,config.AREA_SIZE)
ax.set_ylim(0,config.AREA_SIZE)
ax.set_title(f"UAV Tracking Algorithms: {', '.join(tracking_modes)}")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.grid(True)


for obstacle in obstacles_list:
    circle = plt.Circle(obstacle["center"],obstacle["radius"],color="r",alpha=0.3)
    ax.add_patch(circle)

# Leader Virtualization
leader_line, = ax.plot([],[],"r-",label="Leader UAV Path")
leader_marker, = ax.plot([],[],"ro",markersize=8,label="Leader UAV Path")


# Followers Virtualization

followers_lines = []
followers_markers = []

for i ,label in enumerate(followers_label):
    line,= ax.plot([],[],f"{followers_colors[i]}-",label=f"{label} Path")
    marker, = ax.plot([],[],f"{followers_colors[i]}o",markersize=8,label=label)
    followers_lines.append(line)
    followers_markers.append(marker)



# Position Saving

leader_position = []
followers_position = [[] for _ in range(len(followers))]

# Animation Updating

def update(frame):
    print(frame)
    global leader_position,followers_position
    leader_pos = leader.update(config.DT)
    #  Maybe can be adding type checking system
    if not isinstance(leader_pos,np.ndarray) or leader_pos.shape !=(2,):
        print(f"Fixing leader.update() {leader_pos},{leader_pos.shape}")
        leader_pos = np.array([0.0,0.0])
    leader_position.append(leader_pos)

    leader_line.set_data(
        np.array(leader_position)[:,0],
        np.array(leader_position)[:,1]
    )
    leader_marker.set_data([leader_pos[0]],[leader_pos[1]])

    for i,f in enumerate(followers):
        f_pos= f.update(config.DT,leader)
        # Same checking function can be added
        if not isinstance(f_pos,np.ndarray) or f_pos.shape !=(2,):
            print(f"Fixing followers[i].update() {f_pos},{f_pos.shape}")
            f_pos = np.array([0.0,0.0])

        followers_position[i].append(f_pos)
        followers_lines[i].set_data(
            np.array(followers_position[i])[:,0],
            np.array(followers_position[i])[:,1]
        )

        followers_markers[i].set_data([f_pos[0]],[f_pos[1]])

    return [leader_line,leader_marker] + followers_lines + followers_markers



    # Animation

ani= animation.FuncAnimation(fig, update,
                             frames=config.NUM_STEPS,
                             interval=10,
                             blit=True)

# Saving Animation
def save_animation():
    file_name="animations/animation_1.mp4"
    i=1
    while os.path.exists(file_name):
        i+=1
        file_name = f"animations/animation_{i}.mp4"
    ani.save(file_name,writer="ffmpeg",fps=10)
    print(f"Animation saved at {file_name}")



# Showing and Saving
save_animation()
plt.legend()
plt.show()


