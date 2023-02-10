import carla
import numpy as np
import datetime
import cv2

array_output = -1
frame_counter = -1
IMG_FILEPATH = 'images/'
detected_crash = False

def spawn_vehicle(world, spawn_transform=None, vehicle_type=None, vehicle_color=None):
    """
    :param spawn_transform: if None, will randomize
    :param vehicle_type: if None, will randomize
    :param vehicle_color: if None, will randomize
    :return: carla vehicle object
    """
    if vehicle_type is None:
        vehicle_names = [v.id[8:] for v in world.get_blueprint_library().filter("vehicle")]
        vehicle_type = np.random.choice(vehicle_names)
    blueprint = world.get_blueprint_library().find("vehicle."+vehicle_type)
    if blueprint.has_attribute('color'):
        if vehicle_color is None:
            vehicle_color = f'{np.random.choice(256)},{np.random.choice(256)},{np.random.choice(256)}'
        blueprint.set_attribute('color', vehicle_color)
    if spawn_transform is None:
        spawn_transform = np.random.choice(world.get_map().get_spawn_points())
    spawn_transform.location.z += 0.1 # helps in case map starting locations are incorrect
    # print(f"{vehicle_type}    loc: {spawn_transform.location}")
    vehicle = world.spawn_actor(blueprint, spawn_transform)
    return vehicle

def spectator_camera_transform(actor):
    """
    For visualization only :: used for spectator
    :param actor: Carla Actor object
    :return:
    """
    base_transform = actor.get_transform()
    heading = base_transform.rotation.get_forward_vector()
    x, y, z = base_transform.location.x, base_transform.location.y, base_transform.location.z
    right_vec = base_transform.rotation.get_right_vector()
    camera_location = carla.Location(x, y, z)  - 4*heading #- 5*right_vec
    camera_location.z += 2
    pitch, yaw, roll = base_transform.rotation.pitch, base_transform.rotation.yaw, base_transform.rotation.roll
    camera_rotation = carla.Rotation(pitch-15, yaw, roll)
    return carla.Transform(camera_location, camera_rotation)

def spawn_rgb_sensor(world, actor, resolution=(1280,1280), height_offset=0.2, pitch=-15, filepath=IMG_FILEPATH):
    """
    Camera is located *ON TOP* of vehicle (5/8 of the way to the front)
    (for cars with long hoods, this might be in an unrealistic place)
    :param actor: carla (ego) vehicle
    :param resolution: tuple (res_x, res_y)
    :return: camera actor
    """
    global frame_counter
    frame_counter = -1
    blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
    blueprint.set_attribute('image_size_x', str(resolution[0]))
    blueprint.set_attribute('image_size_y', str(resolution[1]))
    height = 2 * actor.bounding_box.extent.z
    length = 2 * actor.bounding_box.extent.x
    camera_transform = carla.Transform(carla.Location(length/8, 0, height + height_offset),
                                       carla.Rotation(pitch,0,0))
    camera = world.spawn_actor(blueprint, camera_transform, attach_to=actor)
    # camera.listen(lambda data: sensor_save_callback(data, filepath))  # save callback
    camera.listen(lambda data: sensor_array_callback(data))
    return camera


def sensor_array_callback(sensor_data):
    global array_output
    global frame_counter
    img_arr = np.ndarray(shape=(sensor_data.height, sensor_data.width, 4), dtype=np.uint8,
                         buffer=sensor_data.raw_data)  # RGBA format
    array_output = img_arr[:,:,:3] # keep only RGB
    array_output = cv2.cvtColor(array_output, cv2.COLOR_BGR2RGB)
    frame_counter += 1


def sensor_save_callback(sensor_data, filepath):
    exp_time = datetime.datetime.now()
    exp_time_str = f'{exp_time.month}-{exp_time.day}_{exp_time.hour}{exp_time.minute}_{exp_time.second}-{exp_time.microsecond//1000}'
    sensor_data.save_to_disk(f'{filepath}test_{exp_time_str}')
    # img_arr = np.ndarray(shape=(sensor_data.height, sensor_data.width, 4), dtype=np.uint8, buffer=sensor_data.raw_data)  # RGBA format
    # img_arr = img_arr[:,:,:3] # keep only RGB
    # sensor_queue.put(sensor_data.frame)


def spawn_collision_sensor(world, actor):
    sensor_bp = world.get_blueprint_library().find('sensor.other.collision')
    sensor = world.spawn_actor(sensor_bp, carla.Transform(), attach_to=actor)
    sensor.listen(lambda event: crash_detector_callback(event))
    return sensor

def crash_detector_callback(event):
    global detected_crash
    detected_crash = True