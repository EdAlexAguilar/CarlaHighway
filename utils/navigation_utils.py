import numpy as np

def next_n_items(item_list, start_id, n):
    """returns n items with wrap-around"""
    if start_id + n > len(item_list):
        return item_list[start_id:] + item_list[:n-(len(item_list)-start_id)]
    else:
        return item_list[start_id:start_id + n]


class NavigationWaypoints:
    def __init__(self, carla_map, road_list, lane_list, waypoint_dist=0.5):
        """
        navigation_points is a dict per lane, holding a list of carla.location
        """
        self.carla_map = carla_map
        self.road_list = road_list
        self.lane_list = lane_list
        self.waypoint_dist = waypoint_dist
        self.carla_waypoints = carla_map.generate_waypoints(waypoint_dist)
        self.navigation_points = self.create_navigation_points()

    def create_navigation_points(self):
        nav_points = {}
        for lane in self.lane_list:
            nav_points[lane] = self.location_waypoints_for_lane(lane)
        return nav_points

    def location_waypoints_for_lane(self, lane):
        navigation_waypoints = [w.transform.location for w in self.carla_waypoints
                                if w.road_id == self.road_list[0] and w.lane_id == lane]
        navigation_waypoints.reverse() # todo: depends on direction of travel
        for road_num in self.road_list[1:]:
            road_wp = [w.transform.location for w in self.carla_waypoints
                       if w.road_id == road_num and w.lane_id == lane]
            last_nav_point = navigation_waypoints[-1]
            if last_nav_point.distance(road_wp[-1]) < last_nav_point.distance(road_wp[0]):
                road_wp.reverse()
            navigation_waypoints = navigation_waypoints + road_wp
        return navigation_waypoints

    def get_new_points(self, location, lane, num=500):
        distances = np.array([location.distance(w) for w in self.navigation_points[lane]])
        closest_id = distances.argmin()
        return next_n_items(self.navigation_points[lane], closest_id, n=num)

