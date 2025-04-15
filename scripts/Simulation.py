# Simulation.py:
# Spawn all the cars ( similar to the demo ) 
# For each car
#     attach lidar sensor (for logging)
#     if evil:
#         uses attacker class to determine spoofing/removal behavior
#     else (if good): 
#         perform normal behavior
# Perform simulation without firewall, logging for evaluation, adjusts affinity scores
# Perform simulation WITH firewall, logging for evaluation, using affinity scores from before
import carla
import os
import random
import time
import subprocess

from Car import Car

# Run CARLA
cwd = r"D:\CARLA\CARLA_0.9.15\WindowsNoEditor"
port = 2000

exe_path = os.path.join(cwd, "CarlaUE4.exe")

subprocess.Popen([exe_path, '-quality-level=Low', f'-carla-port={port}'])
time.sleep(5)

# Connect to the CARLA server
client = carla.Client('localhost', port)
client.set_timeout(60.0)
world = client.get_world()
traffic_manager = client.get_trafficmanager()
tm_port = traffic_manager.get_port()

# spawning in a random car
car1 = Car()
car1.build_car(world)

spectator = world.get_spectator()
transform = car1.vehicle.get_transform()
spectator.set_transform(carla.Transform(
    transform.location + carla.Location(z=50),  # Move camera up
    carla.Rotation(pitch=-90)                  # Look straight down
))