 #!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import random
import carla

# Append the CARLA egg file to the system path for module imports
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# Import pygame for rendering
try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

# Import numpy for image processing
try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# Import queue for data handling
try:
    import queue
except ImportError:
    import Queue as queue

# Append the CARLA Python API path
sys.path.append('C:/CARLA_0.9.14/PythonAPI/carla')

# Import the BasicAgent for autonomous vehicle control
from agents.navigation.basic_agent import BasicAgent 


class CarlaSyncMode(object):
    """
    Context manager to synchronize CARLA simulation with sensors.
    """
    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


def draw_image(surface, image, blend=False):
    """
    Convert a CARLA image to a Pygame surface and draw it.
    """
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


def get_font():
    """
    Get the default font for rendering text in Pygame.
    """
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    """
    Check if the Pygame window should be closed.
    """
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False


def init_pygame():
    """
    Initialize Pygame and return display surface and font.
    """
    pygame.init()
    display = pygame.display.set_mode(
        (800, 600),
        pygame.HWSURFACE | pygame.DOUBLEBUF
    )
    font = get_font()
    return display, font


def create_client():
    """
    Create and return a CARLA client connected to the server.
    """
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    return client


def spawn_vehicle(world, blueprint_library, spawn_point):
    """
    Spawn a vehicle at a given spawn point and return the vehicle actor.
    """
    #blueprint_library = world.get_blueprint_library()                                    .....blueprint for cycle.....
    #vehicle1_blueprints = list(blueprint_library.filter('vehicle.bh.crossbike'))
    #vehicle2_blueprints = list(blueprint_library.filter('vehicle.diamondback.century'))
    #vehicle3_blueprints = list(blueprint_library.filter('vehicle.gazelle.omafiets'))
    #all_blueprints = vehicle1_blueprints + vehicle2_blueprints + vehicle3_blueprints
    #blueprint = random.choice(all_blueprints)
    vehicle_blueprints = list(blueprint_library.filter('vehicle.*.*'))
    blueprint = random.choice(vehicle_blueprints)
    vehicle = world.spawn_actor(blueprint, spawn_point)
    return vehicle


def setup_agent(vehicle, destination, target_speed=50):
    """
    Setup the BasicAgent for the vehicle, set its destination, and target speed.
    """
    agent = BasicAgent(vehicle)
    agent.set_destination(destination)
    agent.set_target_speed(target_speed)
    return agent


def draw_route(world, route_plan):
    """
    Draw the route on the CARLA world using debug strings.
    """
    for wp in route_plan:
        world.debug.draw_string(wp[0].transform.location, '°', draw_shadow=False,
                                color=carla.Color(r=0, g=255, b=255), life_time=150.0,
                                persistent_lines=True)


def spawn_camera(world, blueprint_library, vehicle):
    """
    Spawn and attach an RGB camera to the vehicle.
    """
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    return camera


def clean_up(actor_list):
    """
    Destroy all actors in the provided actor list.
    """
    for actor in actor_list:
        actor.destroy()
    pygame.quit()
    print('done.')


def main():
    """
    Main function to run the CARLA simulation and render the output.
    """
    actor_list = []
    display, font = init_pygame()
    clock = pygame.time.Clock()
    client = create_client()

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    point_a = spawn_points[0]
    point_b = spawn_points[27].location

    vehicle = spawn_vehicle(world, blueprint_library, point_a)
    actor_list.append(vehicle)

    target_speed = 50  # Set the target speed to 50 km/h
    agent = setup_agent(vehicle, point_b, target_speed)
    route_plan = agent.get_local_planner().get_plan()
    draw_route(world, route_plan)

    camera_rgb = spawn_camera(world, blueprint_library, vehicle)
    actor_list.append(camera_rgb)

    with CarlaSyncMode(world, camera_rgb, fps=30) as sync_mode:
        while True:
            if should_quit():
                break
            clock.tick()

            snapshot, image_rgb = sync_mode.tick(timeout=2.0)
            control = agent.run_step()
            vehicle.apply_control(control)

            # Get the vehicle speed in km/h
            velocity = vehicle.get_velocity()
            speed = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5

            draw_image(display, image_rgb)
            display.blit(
                font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                (8, 10)
            )
            display.blit(
                font.render('Speed: %.2f km/h' % speed, True, (255, 255, 255)),
                (8, 30)
            )
            pygame.display.flip()

    clean_up(actor_list)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
