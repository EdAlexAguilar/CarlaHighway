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


# for spawning
init_road = np.random.choice(CONST.TRACK_ROADS)
waypoints = [w for w in carla_map.generate_waypoints(2)
             if w.road_id==init_road and w.lane_id == 4]
waypoints.reverse()

def main(actors, controllers):
    world.tick()
    #t=0
    while True:
        try:
            #t += 1
            for control in controllers:
                control.update()
                #if t % 100 == 0:
                #    rand_delta = np.random.randint(3) - 1
                #    control.change_lane(rand_delta)
            world.get_spectator().set_transform(c_utils.spectator_camera_transform(actors[len(actors)//2]))
            world.tick()
            continue
        except KeyboardInterrupt:
            print('\n Destroying all Actors')
            for npc in actors:
                carla.command.DestroyActor(npc)
            client.reload_world()
            break

def spawn_controllers(actors, target_speed):
    controllers = []
    for actor in actors:
        actor_info = CONST.vehicles[actor.type_id[8:]]
        control_params = {"kv": actor_info.pp_kv,
                          "kc": actor_info.pp_kc,
                          "speed_kp": actor_info.speed_kp,
                          "speed_ki": actor_info.speed_ki,
                          "speed_kd": actor_info.speed_kd,
                          "target_speed": target_speed}
        control = pp.PurePursuit(actor, navpoints, carla_map, **control_params)
        controllers.append(control)
    return controllers

def new_random_waypoint(waypoint, min_offset=1, rand_offset=10, same_lane_offset=5):
    new_lane = np.random.choice(CONST.TRACK_LANES)
    new_wp = waypoint.next(min_offset + rand_offset*np.random.rand())[0]
    if new_lane == waypoint.lane_id:
        new_wp = new_wp.next(same_lane_offset)[0]
    while new_lane < new_wp.lane_id:
        new_wp = new_wp.get_left_lane()
    while new_lane > new_wp.lane_id:
        new_wp = new_wp.get_right_lane()
    return new_wp

def spawn_actors(num_actors, world, actors, candidates, last_waypoint):
    wp = last_waypoint
    for _ in range(num_actors):
        actor_model = np.random.choice(candidates)
        wp = new_random_waypoint(wp)
        vehicle = c_utils.spawn_vehicle(world, wp.transform, actor_model)
        actors.append(vehicle)
    return actors, wp

if __name__ == "__main__":
    ego_wp = waypoints[0]
    actors, _ = spawn_actors(40, world, [], list(CONST.vehicles.keys()), ego_wp)
    world.tick()
    controllers = spawn_controllers(actors, 20)

    """
    actors = []
    Merc = CONST.merc

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
    """
    main(actors, controllers)