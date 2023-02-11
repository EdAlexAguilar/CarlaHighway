import carla
import numpy as np
from utils.pid import PID

class PurePursuit():
    """
    Waypoints are ordered (carla.Waypoint objects)
    """
    def __init__(self, vehicle, navigation_points, carla_map,
                 kv=2, kc=3, delta_mul=2,
                 speed_kp=0.38, speed_kd=0.2, speed_ki=0,
                 target_speed=15, min_num_waypoints=100):
        self.vehicle = vehicle
        self.carla_map = carla_map
        self.navpoints = navigation_points # Navigation Class that can fetch new points
        # initial set of waypoints
        self.waypoints = self.navpoints.get_new_points(self.vehicle.get_location(),
                                                       self.carla_map.get_waypoint(self.vehicle.get_location()).lane_id,
                                                       num=500) # list of Vector3D
        # self.historical_waypoints = waypoints # list of all waypoints ever given
        self.min_num_waypoints = min_num_waypoints  # gets new waypoints if below this number
        self.kv = kv
        self.kc = kc
        self.original_target = target_speed
        self.target_speed = target_speed
        self.frame_counter = 0
        self.lane_change_every = 1025 + np.random.randint(50)
        self.half_length = vehicle.bounding_box.extent.x  - 0.5 # half the vehicle's length correction to put in axle
        self.speed_controller = PID(kp=speed_kp, kd=speed_kd, ki=speed_ki, base_control=0.45)
        self.delta_mul = delta_mul
        # self.cumulative_error = 0  # for evaluation and tuning of controller

    def append_waypoints(self, new_waypoints):
        self.waypoints = self.waypoints + new_waypoints
        # self.historical_waypoints = self.historical_waypoints + new_waypoints

    def change_lane(self, delta_lane):
        current_lane = self.carla_map.get_waypoint(self.vehicle.get_location()).lane_id
        if (current_lane + delta_lane) in self.navpoints.lane_list:
            self.waypoints = self.navpoints.get_new_points(self.vehicle.get_location(),
                                                           current_lane + delta_lane,
                                                           num=500)

    def longitudinal_control(self):
        measured_speed = self.vehicle.get_velocity().length()
        throttle_control = self.speed_controller.control(self.target_speed, measured_speed)
        throttle_control = np.clip(throttle_control, 0, 1)
        return throttle_control

    def lateral_control(self):
        speed = self.vehicle.get_velocity()
        speed = speed.length()
        lookahead = self.kv * speed + self.kc

        transform = self.vehicle.get_transform()
        location = transform.location
        heading = transform.rotation.get_forward_vector()
        rear_axle_loc = location - self.half_length * heading

        distances = np.array([(rear_axle_loc.distance(w) - lookahead) for w in self.waypoints])
        target_id = np.abs(distances).argmin()
        self.waypoints = self.waypoints[target_id:]
        v_target = self.waypoints[0] - rear_axle_loc
        v_target_length = v_target.length()

        v_target_unitary = v_target.make_unit_vector()
        sin_alpha = v_target_unitary.cross(heading)
        sign_sin_alpha = np.sign(sin_alpha.dot(carla.Vector3D(z=1)))
        sin_alpha = sin_alpha.length() * sign_sin_alpha
        delta = - np.arctan(4 * self.half_length * sin_alpha / v_target_length)
        delta *= 2 / np.pi
        delta *= self.delta_mul

        if len(self.waypoints) < self.min_num_waypoints:
            lane = self.carla_map.get_waypoint(self.vehicle.get_location()).lane_id
            if lane in self.navpoints.navigation_points.keys():
                self.waypoints = self.navpoints.get_new_points(self.vehicle.get_location(),
                                                               lane,
                                                               num=500)

        # closest_historical = np.array([rear_axle_loc.distance(w) for w in self.historical_waypoints])
        # closest_historical = closest_historical.argmin()
        # self.cumulative_error += rear_axle_loc.distance(self.historical_waypoints[closest_historical])
        return delta

    def update(self):
        self.frame_counter += 1
        if self.frame_counter % self.lane_change_every == 0:
            self.change_lane(np.random.randint(3) - 1)
        control_command = carla.VehicleControl()
        control_command.throttle = self.longitudinal_control()
        control_command.steer = self.lateral_control()
        self.vehicle.apply_control(control_command)
