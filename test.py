import carla
import random
import time
import subprocess

# Run CARLA
cwd = 'C:/Users/jrr77/Documents/Github/CSS-CAV' # Change to .env? so we don't all have to adjust
port = 2000
subprocess.Popen(['CarlaUE4.exe', '-quality-level=Low', f'-carla-port={port}'], cwd=cwd)
time.sleep(5)  # Wait for the simulator to start

# Connect to the CARLA server
client = carla.Client('localhost', port)
client.set_timeout(60.0)
world = client.get_world()
traffic_manager = client.get_trafficmanager()
tm_port = traffic_manager.get_port()

# Load the car models
blueprint_library = world.get_blueprint_library()
vehicles_bps = []

# Filter for vehicle blueprints
for bp in blueprint_library.filter('vehicle.*'):
    vehicles_bps.append(bp)
    
print("Found {} vehicle blueprints.".format(len(vehicles_bps)))
print("-------------------------------------------------")

# Randomly choose two spawn points
spawn_points = world.get_map().get_spawn_points()

# Spawn the cars and wait for 10 seconds
for bp in vehicles_bps:
    # Check if there are any spawn points left
    if spawn_points == []:
        break 
    spawn_point = random.choice(spawn_points)
    spawn_points.remove(spawn_point)
    vehicle = world.spawn_actor(bp, spawn_point)
    print("Spawned vehicle {} at {}".format(vehicle.id, spawn_point))
    
print("Spawned all vehicles. Waiting 3 seconds...")
print("-------------------------------------------------")    
time.sleep(3)

# Start autopilot and control the cars
for vehicle in world.get_actors().filter('vehicle.*'):
    vehicle.set_autopilot(True)
     # Randomly assign car as malicous
    if random.random() > 1:
        print(vehicle.id, "is NOT evil...")
    else:
        print(vehicle.id, "is evil!")
        traffic_manager.ignore_lights_percentage(vehicle, 100.0)
        traffic_manager.distance_to_leading_vehicle(vehicle, -20.0)
        traffic_manager.vehicle_percentage_speed_difference(vehicle, -500.0)
        traffic_manager.ignore_vehicles_percentage(vehicle, 100.0)

print("Running vehicles in autopilot for 120 seconds...")
print("-------------------------------------------------")
# Set up a collision sensor for each vehicle
collision_sensors = []

def log_collision(event):
    actor = event.actor
    other_actor = event.other_actor
    impulse = event.normal_impulse
    print(f"Collision detected! {actor.type_id} collided with {other_actor.type_id} with impulse {impulse}.")

for vehicle in world.get_actors().filter('vehicle.*'):
    collision_sensor_bp = blueprint_library.find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_sensor_bp, carla.Transform(), attach_to=vehicle)
    collision_sensor.listen(log_collision)
    collision_sensors.append(collision_sensor)
    
time.sleep(120)

# Destroy the cars
for vehicle in world.get_actors().filter('vehicle.*'):
    vehicle.destroy()
    
print("Destroyed all vehicles. Goodbye!")