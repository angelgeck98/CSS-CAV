import carla
import random
import time

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# Load the car models
blueprint_library = world.get_blueprint_library()
vehicle1_bp = blueprint_library.find('vehicle.tesla.model3')
vehicle2_bp = blueprint_library.find('vehicle.audi.a2')

# Randomly choose two spawn points
spawn_points = world.get_map().get_spawn_points()
spawn_point1 = random.choice(spawn_points)
spawn_point2 = carla.Transform(spawn_point1.location + carla.Location(x=-5, y=0, z=0), spawn_point1.rotation)

# Spawn the cars and wait for 10 seconds
vehicle1 = world.spawn_actor(vehicle1_bp, spawn_point1)
vehicle2 = world.spawn_actor(vehicle2_bp, spawn_point2)
time.sleep(10)

# Start autopilot and control the cars for 30 seconds
vehicle1.set_autopilot(True)
vehicle1.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))

vehicle2.set_autopilot(True)
vehicle2.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
time.sleep(30)

# Destroy the cars
vehicle1.destroy()
vehicle2.destroy()