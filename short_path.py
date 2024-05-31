import carla
import time
import cv2
import numpy as np
import random
import sys
sys.path.append('C:/CARLA_0.9.14/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner
# define speed contstants
PREFERRED_SPEED = 40
SPEED_THRESHOLD =2 

def connect_to_client(host='localhost', port=2000):
    return carla.Client(host, port)

def get_spawn_points(world):
    return world.get_map().get_spawn_points()

def select_blueprint(world):                                                                               #Blueprint Selection Function:
    blueprint_library = world.get_blueprint_library()
    vehicle1_blueprints = list(blueprint_library.filter('vehicle.bh.crossbike'))
    vehicle2_blueprints = list(blueprint_library.filter('vehicle.diamondback.century'))
    vehicle3_blueprints = list(blueprint_library.filter('vehicle.gazelle.omafiets'))
    all_blueprints = vehicle1_blueprints + vehicle2_blueprints + vehicle3_blueprints
    vehicle_bp = random.choice(all_blueprints)
    return vehicle_bp

def spawn_vehicle(world, vehicle_bp, spawn_point):
    return world.try_spawn_actor(vehicle_bp, spawn_point)

def configure_camera_blueprint(world, width=640, height=360):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(width))
    camera_bp.set_attribute('image_size_y', str(height))
    return camera_bp

def spawn_camera(world, camera_bp, vehicle, x=-5, z=3):
    camera_init_trans = carla.Transform(carla.Location(x=x, z=z))
    return world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

def camera_callback(image, data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

def setup_camera_listener(camera, camera_data):
    camera.listen(lambda image: camera_callback(image, camera_data))

def initialize_global_route_planner(world, sampling_resolution=1):
    return GlobalRoutePlanner(world.get_map(), sampling_resolution)



def get_longest_route(grp, point_a,point_b):
    longest_distance = 0
    longest_route = []
    cur_route = grp.trace_route(point_a, point_b)
    if len(cur_route) > longest_distance:
        longest_distance = len(cur_route)
        longest_route = cur_route
    return longest_route


def draw_route(world, route):
    for waypoint in route:
        world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
                                color=carla.Color(r=0, g=0, b=255), life_time=60.0,
                                persistent_lines=True)
        

def move_vehicle_along_route(vehicle, route, camera_data):
    for waypoint in route:
        vehicle.set_transform(waypoint[0].transform)
        cv2.imshow('Fake self-driving', camera_data['image'])
        cv2.waitKey(50)

def cleanup(world, vehicle, camera):
    time.sleep(2)
    cv2.destroyAllWindows()
    camera.stop()
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for sensor in world.get_actors().filter('*sensor*'):
        sensor.destroy()

def main():
    client = connect_to_client()
    world = client.get_world()
    spawn_points = get_spawn_points(world)
    vehicle_bp = select_blueprint(world)
    vehicle = spawn_vehicle(world, vehicle_bp, spawn_points[0])
    
    camera_bp = configure_camera_blueprint(world)
    camera = spawn_camera(world, camera_bp, vehicle)
    
    image_w = camera_bp.get_attribute('image_size_x').as_int()
    image_h = camera_bp.get_attribute('image_size_y').as_int()
    
    camera_data = {'image': np.zeros((image_h, image_w, 4))}
    setup_camera_listener(camera, camera_data)
    
    point_a = spawn_points[0].location
    point_b = spawn_points[87].location
    
    grp = initialize_global_route_planner(world)
    route = get_longest_route(grp, point_a,point_b)
    draw_route(world, route)
    
    move_vehicle_along_route(vehicle, route, camera_data)
    
    cleanup(world, vehicle, camera)

if __name__ == "__main__":
    main()
