import numpy as np
import random
import config
import obstacles
from obstacles import get_exit_direction,is_inside_obstacle,create_obstacles
import tracking_algorithms as tracking



class UAV:
    def __init__(self,speed,acceleration,obstacles_list,avoid_obstacles=True,tracking_modes=None):

        self.pos=np.array([random.uniform(0,config.AREA_SIZE),
                          random.uniform(0,config.AREA_SIZE)])

        self.obstacles_list = obstacles_list if obstacles_list is not None else []
        self.avoid_obstacles_enabled=avoid_obstacles
        self.tracking_modes = tracking_modes if tracking_modes is not None else []
        self.speed= speed
        self.acceleration = acceleration
        self.angle= random.uniform(0,2*np.pi)
        self.avoid_obstacles_enabled=avoid_obstacles
        self.vel= np.array([np.cos(self.angle),np.sin(self.angle)])*self.speed
        self.change_interval = config.CHANGE_INTERVAL
        self.time_since_change = 0

    def update_position(self,dt):

        self.pos += self.vel*dt
        self.pos = np.clip(self.pos,[0,0],[config.AREA_SIZE,config.AREA_SIZE])
    def avoid_obstacles(self, dt):
        if not isinstance(self.obstacles_list, list):
            raise TypeError(f"avoid_obstacles: self.obstacles_list bir liste değil! Tür: {type(self.obstacles_list)}")

        if is_inside_obstacle(self.pos, self.obstacles_list):
            exit_dir = get_exit_direction(self.pos, self.obstacles_list)
            if exit_dir is not None:
                self.pos += exit_dir * self.speed * dt * 2
                self.pos = np.clip(self.pos, [0, 0], [config.AREA_SIZE, config.AREA_SIZE])
                return True
        return False
    def near_boundary(self,margin=config.AREA_SIZE/20):
        return(self.pos[0]<margin or self.pos[0]>config.AREA_SIZE-margin
                                  or
               self.pos[1]<margin or self.pos[1]>config.AREA_SIZE-margin)

    def avoid_boundary(self,dt):
        correction = np.array([0.0,0.0])

        if self.pos[0]<(config.AREA_SIZE/20):
            correction[0]=1

        elif self.pos[0]> config.AREA_SIZE-(config.AREA_SIZE/20):
            correction[0]=-1

        if self.pos[1]<(config.AREA_SIZE/20):
            correction[1]=1
        elif self.pos[1]> config.AREA_SIZE-(config.AREA_SIZE/20):
            correction[1]=-1


        if np.linalg.norm(correction)>0:
            correction /= np.linalg.norm(correction)
            self.pos += correction*self.speed*dt*self.acceleration
    def change_direction(self):
        self.angle = random.uniform(0,2*np.pi)
        self.vel =np.array([np.cos(self.angle),np.sin(self.angle)])*self.speed



class Leader(UAV):
    def __init__(self,obstacles_list):
        super().__init__(speed=2,obstacles_list=obstacles,acceleration=1.1,avoid_obstacles=True)


    def update(self, dt):
        if self.near_boundary():
            self.avoid_boundary(dt)

        self.time_since_change += dt
        if self.time_since_change >= self.change_interval:
            self.change_direction()
            self.time_since_change = 0

        self.update_position(dt)


        if not isinstance(self.pos, np.ndarray) or len(self.pos) != 2:
            print(f"Hata: Leader update() yanlış veri döndürdü! {type(self.pos)} {self.pos}")
            self.pos = np.array([0.0, 0.0])

        return np.array(self.pos)
    def predict_future_position(self, future_time):

        future_pos = self.pos + self.vel * future_time
        future_pos = np.clip(future_pos, [0, 0], [config.AREA_SIZE, config.AREA_SIZE])
        return future_pos



class Follower(UAV):
    def __init__(self, obstacles_list,tracking_modes=None):
        super().__init__(speed=1, acceleration=1.1, obstacles_list= obstacles_list,avoid_obstacles=True, tracking_modes=tracking_modes)
        self.path=[]
        self.replan_flag=True
    def update(self,dt,leader):
        if self.avoid_obstacles(dt):
            return self.pos

        if self.near_boundary():
            self.avoid_boundary(dt)
            return self.pos

        if not self.tracking_modes:
            return self.pos

        if not isinstance(leader.pos, np.ndarray) or leader.pos.shape != (2,):
            print(f"Hata: Leader pos yanlış türde! {type(leader.pos)}, düzeltildi.")
            leader.pos = np.array([0.0, 0.0])



        # Waits mode
        for mode in self.tracking_modes:
            if mode in tracking.AVAILABLE_TRACKING_MODES:
                if mode =="astar":
                    tracking.astar_follow(self,dt,leader.pos,self.obstacles_list)

                elif mode == "predictive":
                    tracking.predictive_follow(self,dt,leader)


                elif mode == "direct":
                    tracking.direct_follow(self,dt,leader)


        return np.array(self.pos)





