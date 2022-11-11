import carla
import numpy as np
from collections import namedtuple
import utils.constants as CONST
import utils.carla_utils as c_utils
import utils.pure_pursuit as pp


client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
world.set_weather(carla.WeatherParameters().ClearSunset)
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

carla_map = world.get_map()


"""
ROAD NETWORK -- # todo clean
"""
def create_location_waypoints(road_list, lane=5):
    """
    :param road_list: road_id (int) list
    """
    navigation_waypoints = [w.transform.location for w in carla_map.generate_waypoints(WAYPOINT_DIST)
                            if w.road_id==road_list[0] and w.lane_id==lane]
    navigation_waypoints.reverse()
    for road_num in road_list[1:]:
        road_wp = [w for w in carla_map.generate_waypoints(WAYPOINT_DIST)
                if w.road_id==road_num and w.lane_id==lane]
        road_wp = [w.transform.location for w in road_wp]
        last_nav_point = navigation_waypoints[-1]
        if last_nav_point.distance(road_wp[-1]) < last_nav_point.distance(road_wp[0]):
            road_wp.reverse()
        navigation_waypoints = navigation_waypoints + road_wp
    return navigation_waypoints

WAYPOINT_DIST = 1
# This list of roads corresponds to the 8-shaped loop
TRACK_ROADS = [1092, 38, 1601, 37, 761, 36, 862, 35, 43, 266, 42, 50, 1174, 49, 902, 48, 775, 47, 1073, 46, 144, 45, 6,
                   41, 1400, 40,]# 1185, 39, 1092, 38, 1601, 37, 761]
waypoints = [w for w in carla_map.generate_waypoints(WAYPOINT_DIST)
             if w.road_id==TRACK_ROADS[0] and w.lane_id == 4]
waypoints.reverse()
navigation_waypoints = create_location_waypoints(TRACK_ROADS)





""" 
Spawn actor and test driving
"""

NPC = namedtuple('NPC', ['model', 'velocity', # carla model name, velocity type
                         'speed_kp', 'speed_ki', 'speed_kd', # PID throttle
                         'pp_kv', 'pp_kc']) # pure pursuit velocity/constant terms



def main():
    world.tick()
    t=0
    while True:
        try:
            t += 1
            control.update()
            world.get_spectator().set_transform(c_utils.spectator_camera_transform(npc))
            world.tick()
            if t%5==0:
                npc_speed = npc.get_velocity().length()
                print(f'{npc_speed:.2f} m/s')
            continue
        except KeyboardInterrupt:
            print('\n Destroying all Actors')
            carla.command.DestroyActor(npc)
            client.reload_world()
            break

if __name__ == "__main__":
    Merc = NPC("mercedes.coupe_2020", 1,
               0.4, 0.0, 0.1,
               1.5, 0.5)
    npc_spawn_transform = waypoints[0].transform
    npc = c_utils.spawn_vehicle(world, npc_spawn_transform, Merc.model)
    world.get_spectator().set_transform(npc_spawn_transform)
    params = {"kv": Merc.pp_kv,
              "kc": Merc.pp_kc,
              "delta_mul": 2,
              "speed_kp": Merc.speed_kp,
              "speed_ki": Merc.speed_ki,
              "speed_kd": Merc.speed_kd,
              "target_speed": 15}
    control = pp.PurePursuit(npc, navigation_waypoints, **params)
    main()