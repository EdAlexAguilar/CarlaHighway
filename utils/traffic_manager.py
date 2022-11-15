

class TrafficManager:
    def __init__(self, actors, controllers, carla_map, speed_limit=25):
        self.actors = actors
        self.controllers = controllers
        self.carla_map = carla_map
        self.speed_limit = speed_limit

    def update_all(self):
        for actor, control in zip(self.actors, self.controllers):
            
            control.update()

