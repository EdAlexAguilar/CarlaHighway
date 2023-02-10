import gym
import numpy as np
import utils.constants as constants

class CarlaHighwayEnv(gym.Env):
    def __init__(self, traffic_speed=14,
                       traffic_density=0.2,
                       vehicle_config=None):
        self.traffic_speed = traffic_speed
        self.traffic_density = traffic_density
        self.vehicle_config = constants.base_vehicle_config
        if vehicle_config is not None:
            for key, value in vehicle_config.items():
                self.vehicle_config[key] = value
        self.observation_space = gym.spaces.Discrete(4)
        self.action_space = gym.spaces.Discrete(4)
        self.spawn_all_actors()
