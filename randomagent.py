"""
Example code to debug and show the environment working
"""
from CarlaHighwayEnv import CarlaHighwayEnv
import numpy as np

env_settings = {"traffic_speed" : 14,  # m/s
                "traffic_density": 0.1,
                "vehicle_config": None,
                "max_num_steps": 2000}

env = CarlaHighwayEnv(**env_settings)

def random_action():
    return 2*np.random.rand(2) - 1

for i in range(env_settings['max_num_steps']):
    action = random_action()
    obs, reward, done, info = env.step(action)
    if done:
        print(f" Vehicle Crashed after {i} steps!")
        env.reset_carla()
        break