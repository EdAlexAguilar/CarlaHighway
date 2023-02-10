from collections import namedtuple
# Possible spawn roads
TRACK_ROADS = [1092, 38, 1601, 37, 761, 36, 862, 35, 43, 266, 42, 50, 1174,
               49, 902, 48, 775, 47, 1073, 46, 144, 45, 6, 41, 1400, 40, 1185,
               39]

TRACK_LANES = [3, 4, 5, 6]

# Vehicles to be used
npcInfo = namedtuple('npcInfo', ['model', # carla model name, velocity type
                         'speed_kp', 'speed_ki', 'speed_kd', # PID throttle
                         'pp_kv', 'pp_kc']) # pure pursuit velocity/constant lookahead terms

vehicles = {"mercedes.coupe_2020": npcInfo("mercedes.coupe_2020",
                                            0.4, 0.0, 0.1,
                                            1.2, 0.5)}
IMG_DIM = 640
base_vehicle_config ={ "image_source": "rgb_camera",
                        "rgb_camera": (IMG_DIM, IMG_DIM)}