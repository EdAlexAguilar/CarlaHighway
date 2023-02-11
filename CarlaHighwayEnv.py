import gym
import numpy as np
import carla
import utils.constants as CONST
import utils.navigation_utils as navigation_utils
import utils.carla_utils as carla_utils
import utils.pure_pursuit as pp
import time

class CarlaHighwayEnv(gym.Env):
    def __init__(self, traffic_speed=14,
                       traffic_density=0.15,
                       vehicle_config=None,
                       max_num_steps=2000):
        self.traffic_speed = traffic_speed
        self.traffic_density = traffic_density
        self.vehicle_config = CONST.base_vehicle_config
        if vehicle_config is not None:
            for key, value in vehicle_config.items():
                self.vehicle_config[key] = value
        self.observation_space = gym.spaces.Discrete(4)
        self.action_space = gym.spaces.Discrete(2)
        self.candidates = CONST.vehicles  # dictionary containing possible cars to spawn for other actors
        self.ego_model = "mercedes.coupe_2020"
        self.time_delta = 0.05
        self.waypoint_dist = 10
        self.max_num_steps = max_num_steps  # episode ends if limit is reached
        self.connect_to_carla()
        _ = self.reset()

    def reset(self):
        self.prev_throttle = 0
        self.prev_steer = 0
        self.prev_yaw = 0
        self.reset_carla()
        self.spawn_ego_and_actors() # contains world.tick()
        self.current_frame = 0
        carla_utils.frame_counter = 0
        carla_utils.detected_crash = False
        obs = self.get_ego_obs()
        return obs

    def step(self, action):
        # basic control of other actors
        for control in self.controllers:
            control.update()
        # control for ego vehicle
        control_command = carla.VehicleControl()
        control_command.throttle = action[0]
        control_command.steer = action[1]
        self.ego_vehicle.apply_control(control_command)
        # tick world and move spectator for visualization
        self.world.tick()
        self.current_frame += 1
        self.world.get_spectator().set_transform(carla_utils.spectator_camera_transform(self.ego_vehicle))
        # update waypoints for ego
        self.ego_waypoints = self.ego_navpoints.get_new_points(self.ego_vehicle.get_location(),
                                                               self.carla_map.get_waypoint(
                                                                   self.ego_vehicle.get_location()).lane_id,
                                                               num=3)
        # Retrieve data to send back
        timeout = self.current_frame > self.max_num_steps
        obs = self.get_ego_obs(action)
        done = obs['collision'] or timeout
        info = {'frame': self.current_frame}
        reward = 1 if not obs['collision'] else -10
        self.prev_throttle = action[0]
        self.prev_steer = action[1]
        return obs, reward, done, info

    def close(self):
        self.reset_carla()

    def connect_to_carla(self):
        """Initial connecting to carla and loading the map"""
        self.client = carla.Client('localhost', 2000)
        self.world = self.client.load_world('Town04')
        self.world.set_weather(carla.WeatherParameters().ClearSunset)
        self.carla_settings = self.world.get_settings()
        self.carla_settings.synchronous_mode = True
        self.carla_settings.fixed_delta_seconds = self.time_delta
        self.world.apply_settings(self.carla_settings)
        self.carla_map = self.world.get_map()

    def reset_carla(self):
        """Destroys all actors and creates a new instance of world"""
        self.client.reload_world(self, reset_settings=False)
        self.world = self.client.get_world()
        self.world.apply_settings(self.carla_settings)
        self.carla_map = self.world.get_map()

    def spawn_ego_and_actors(self):
        self.spawn_ego()  # contains world.tick()
        self.spawn_actors()  # contains world.tick()
        self.world.get_spectator().set_transform(carla_utils.spectator_camera_transform(self.ego_vehicle))
        self.camera = carla_utils.spawn_rgb_sensor(self.world, self.ego_vehicle, self.vehicle_config['rgb_camera'])
        self.collision_sensor = carla_utils.spawn_collision_sensor(self.world, self.ego_vehicle)
        self.world.tick() # to move the camera in place

    def get_ego_obs(self, action):
        obs = {"state" : self.get_ego_state(action)}
        sensor_info = self.get_sensor_info()
        obs = {**obs, **sensor_info}
        return obs

    def get_sensor_info(self):
        sensor_obs = {}
        # extract image from camera sensor
        while self.current_frame != carla_utils.frame_counter:
            time.sleep(0.002)
        camera_image = carla_utils.array_output
        sensor_obs['image'] = camera_image
        # extract collision information
        crash = carla_utils.detected_crash
        sensor_obs['collision'] = crash
        return sensor_obs

    def get_ego_state(self, action):
        """
        Following metadrive format
        https://github.com/metadriverse/metadrive/blob/main/metadrive/obs/state_obs.py#L9
        """
        def vec_to_np(carla_vec):
            x, y, z = carla_vec.x, carla_vec.y, carla_vec.z
            return np.array([x, y, z])
        # ego vehicle info
        speed = self.vehicle.get_velocity()
        speed = speed.length()
        transform = self.ego_vehicle.get_transform()
        rotation = transform.rotation
        heading = vec_to_np(rotation.get_forward_vector())
        yaw = rotation.yaw
        location = vec_to_np(transform.location)
        # road info
        road_waypoint = self.carla_map.get_waypoint(location)
        current_lane = int(road_waypoint.lane_id)
        lane_width = road_waypoint.lane_width
        road_transform = road_waypoint.get_transform()
        road_heading = vec_to_np(road_transform.get_forward_vector())
        road_right = vec_to_np(road_transform.get_right_vector())
        road_location = vec_to_np(road_transform.location)
        dist_vector = location - road_location
        right_side = sign(dist_vector @ road_right)
        dist_to_center = right_side * np.linalg.norm(dist_vector)
        dist_left_line = (current_lane - min(CONST.TRACK_LANES)) * lane_width + lane_width/2 + dist_to_center
        dist_right_line = (max(CONST.TRACK_LANES) - current_lane) * lane_width - lane_width/2 - dist_to_center
        diff_angular_heading_lane = right_side * np.arccos(road_heading @ heading)
        # navigation info  (Sorry for *bad* style!)
        next_waypoint = self.ego_waypoints[1]
        next_diff = next_waypoint - location
        lon_dist_next_wp = heading @ next_diff
        lat_dist_next_wp = next_diff - lon_dist_next_wp*heading
        next_next_waypoint = self.ego_waypoints[2]
        next_next_diff = next_next_waypoint - location
        lon_dist_next_next_wp = heading @ next_next_diff
        lat_dist_next_next_wp = next_next_diff - lon_dist_next_next_wp * heading
        # putting all together
        ego_state = [dist_left_line,
                     dist_right_line,
                     diff_angular_heading_lane,
                     speed,
                     action[1],  # current steering
                     self.prev_throttle,
                     self.prev_steer,
                     (yaw-self.prev_yaw)/self.time_delta,
                     dist_to_center,
                     lon_dist_next_wp,
                     lat_dist_next_wp,
                     0,0,0,
                     lon_dist_next_next_wp,
                     lat_dist_next_next_wp,
                     0,0,0
                    ]
        self.prev_yaw = yaw
        return np.array(ego_state)

    def spawn_actors(self):
        def new_random_waypoint(waypoint, min_offset=3, rand_offset=10, same_lane_offset=7):
            new_lane = np.random.choice(CONST.TRACK_LANES)
            new_wp = waypoint.next(min_offset + rand_offset * np.random.rand())[0]
            if new_lane == waypoint.lane_id:
                new_wp = new_wp.next(same_lane_offset)[0]
            while new_lane < new_wp.lane_id:
                new_wp = new_wp.get_left_lane()
            while new_lane > new_wp.lane_id:
                new_wp = new_wp.get_right_lane()
            return new_wp
        #-------- spawn actors -------------
        wp = self.ego_init_waypoint
        num_actors = int(self.traffic_density * 100)
        self.actors = []
        for _ in range(num_actors):
            actor_model = np.random.choice(list(self.candidates.keys()))
            wp = new_random_waypoint(wp)
            vehicle = c_utils.spawn_vehicle(self.world, wp.transform, actor_model)
            self.actors.append(vehicle)
        self.world.tick()  # before creating the controllers, it's necessary to world.tick()
        # -------- create controllers for actors -------------
        self.navpoints = navigation_utils.NavigationWaypoints(self.carla_map, CONST.TRACK_ROADS, CONST.TRACK_LANES)
        self.controllers = []
        for actor in self.actors:
            actor_info = CONST.vehicles[actor.type_id[8:]]
            control_params = {"kv"          : actor_info.pp_kv,
                              "kc"          : actor_info.pp_kc,
                              "speed_kp"    : actor_info.speed_kp,
                              "speed_ki"    : actor_info.speed_ki,
                              "speed_kd"    : actor_info.speed_kd,
                              "target_speed": self.traffic_speed}
            controller = pp.PurePursuit(actor, self.navpoints, self.carla_map, **control_params)
            self.controllers.append(controller)

    def spawn_ego(self):
        init_road = np.random.choice(CONST.TRACK_ROADS)
        init_lane = np.random.choice(CONST.TRACK_LANES)
        self.ego_init_waypoint = [w for w in carla_map.generate_waypoints(5)
                                if w.road_id == init_road and w.lane_id == init_lane][0]
        self.ego_vehicle = carla_utils.spawn_vehicle(self.world, self.ego_init_waypoint.transform, self.ego_model)
        self.world.tick()
        # navpoints are generated once and cover whole map
        self.ego_navpoints = navigation_utils.NavigationWaypoints(self.carla_map, CONST.TRACK_ROADS, CONST.TRACK_LANES, waypoint_dist=self.waypoint_dist)
        # waypoints are about current plan and are updated constantly
        self.ego_waypoints = self.ego_navpoints.get_new_points(self.ego_vehicle.get_location(),
                                                       self.carla_map.get_waypoint(self.ego_vehicle.get_location()).lane_id,
                                                       num=3)

