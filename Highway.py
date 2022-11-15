import carla
import numpy as np
from collections import namedtuple
import utils.constants as CONST
import utils.carla_utils as c_utils
import utils.pure_pursuit as pp
from utils import navigation_utils


client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
world.set_weather(carla.WeatherParameters().ClearSunset)
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

carla_map = world.get_map()


"""
ROAD NETWORK 
"""
navpoints = navigation_utils.NavigationWaypoints(carla_map, CONST.TRACK_ROADS, CONST.TRACK_LANES)



""" 
Spawn actor and test driving
"""
NPC = namedtuple('NPC', ['model', 'velocity', # carla model name, velocity type
                         'speed_kp', 'speed_ki', 'speed_kd', # PID throttle
                         'pp_kv', 'pp_kc']) # pure pursuit velocity/constant lookahead terms

# for spawning
waypoints = [w for w in carla_map.generate_waypoints(2)
             if w.road_id==1092 and w.lane_id == 4]
waypoints.reverse()

def main(actors, controllers):
    world.tick()
    t=0
    while True:
        try:
            t += 1
            for control in controllers:
                control.update()
                if t % 100 == 0:
                    rand_delta = np.random.randint(3) - 1
                    control.change_lane(rand_delta)
            world.get_spectator().set_transform(c_utils.spectator_camera_transform(actors[0]))
            world.tick()
            continue
        except KeyboardInterrupt:
            print('\n Destroying all Actors')
            for npc in actors:
                carla.command.DestroyActor(npc)
            client.reload_world()
            break


if __name__ == "__main__":
    actors = []
    Merc = NPC("mercedes.coupe_2020", 1,
               0.4, 0.0, 0.1,
               1.2, 0.5)

    npc_spawn_transform = waypoints[0].transform
    npc = c_utils.spawn_vehicle(world, npc_spawn_transform, Merc.model)
    actors.append(npc)
    params = {"kv": Merc.pp_kv,
              "kc": Merc.pp_kc,
              "speed_kp": Merc.speed_kp,
              "speed_ki": Merc.speed_ki,
              "speed_kd": Merc.speed_kd,
              "target_speed": 27}
    world.tick() # otherwise the get_location doesn't work

    controllers = []
    control = pp.PurePursuit(npc, navpoints, carla_map, **params)
    controllers.append(control)

    main(actors, controllers)