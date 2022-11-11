import carla
import numpy as np
from utils.pid import PID

class PurePursuit():
    """
    Waypoints are ordered (carla.Waypoint objects)
    """
    def __init__(self, vehicle, waypoints, kv=3, kc=10, delta_mul=1,
                 speed_kp=0.38, speed_kd=0.2, speed_ki=0, target_speed=15):
        self.vehicle = vehicle
        self.waypoints = waypoints # list of Vector3D
        self.historical_waypoints = waypoints # list of all waypoints ever given
        self.kv = kv
        self.kc = kc
        self.target_speed = target_speed
        self.half_length = vehicle.bounding_box.extent.x  - 0.5 # half the vehicle's length correction to put in axle
        self.speed_controller = PID(kp=speed_kp, kd=speed_kd, ki=speed_ki, base_control=0.45)
        self.delta_mul = delta_mul

        self.cumulative_error = 0

    def append_waypoints(self, new_waypoints):
        self.waypoints = self.waypoints + new_waypoints
        self.historical_waypoints = self.historical_waypoints + new_waypoints

    def longitudinal_control(self):
        # ridiculous speed controller
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

        closest_historical = np.array([rear_axle_loc.distance(w) for w in self.historical_waypoints])
        closest_historical = closest_historical.argmin()
        self.cumulative_error += rear_axle_loc.distance(self.historical_waypoints[closest_historical])
        return delta

    def update(self):
        control_command = carla.VehicleControl()
        control_command.throttle = self.longitudinal_control()
        control_command.steer = self.lateral_control()
        self.vehicle.apply_control(control_command)
