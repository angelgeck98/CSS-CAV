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
import random
import time
import subprocess

# Run CARLA
cwd = 'C:/Users/jrr77/Documents/Github/CSS-CAV'
port = 2000
subprocess.Popen(['CarlaUE4.exe', '-quality-level=Low', f'-carla-port={port}'], cwd=cwd)
time.sleep(5)

# Connect to the CARLA server
client = carla.Client('localhost', port)
client.set_timeout(60.0)
world = client.get_world()
traffic_manager = client.get_trafficmanager()
tm_port = traffic_manager.get_port()

# 