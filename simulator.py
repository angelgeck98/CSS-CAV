import carla
import subprocess
import random
import time

from scripts.Car import Car          # Kayla's code
from mvp.attack.attacker import Attacker
from mvp.defense.perception_defender import PerceptionDefender
# from evaluation import Evaluation  # Angel's code
# assuming Car uses attacker/defender code from Yaz's code

# Starts the CARLA simulator 

'''
def start_carla(carla_path, port, cwd):
    subprocess.Popen(['CarlaUE4.exe', '-quality-level=Low', f'-carla-port={port}'], cwd=cwd)
    print("Starting CARLA simulator...")
    time.sleep(10)
'''

# Kayla's working CARLA starter
def start_carla(carla_path, port):
    subprocess.Popen([carla_path, '-quality-level=Low', f'-carla-port={port}'])
    print("Starting CARLA simulator...")
    time.sleep(10)

class Simulator:
    def __init__(self, port=2000, run_duration=60):
        self.port = port
        self.run_duration = run_duration
        self.client = None
        self.world = None
        self.cars = []
        # self.evaluation = Evaluation()  # Angel's code, may or may not be needed
        self.collision_sensors = []

	# Initialize the CARLA client and world
    def connect(self):
        self.client = carla.Client('localhost', self.port)
        print(f"Connecting to CARLA server at localhost:{self.port}...")
        self.client.set_timeout(60.0)
        self.world = self.client.get_world()
        print("Successfully connected to CARLA server.")

	# Spawn vehicles in the simulation
    def spawn_vehicles(self):
        print("Spawning vehicles...")
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        # Using basic Car spawn instead of defender bc it's not working
        for i in range(1, 6):
            if not spawn_points:
                break
            spawn_point = spawn_points.pop()
            car = Car()
            vehicle = car.build_car("defend", self.world, spawn_point)
            if vehicle is not None:
                self.cars.append(car)
                print(f"Spawned car{i} at {spawn_point}")

        # Spawn in Attackers
        for i in range(1, 16):
            if not spawn_points:
                break
            spawn_point = spawn_points.pop()
            attack = Attacker()
            vehicle = attack.build_car("attack", self.world, spawn_point)
            if vehicle is not None:
                self.cars.append(attack)
                print(f"Spawned attacker{i} at {spawn_point}")

        
        '''
        # Spawn in Defenders
        for i in range(1, 6):
            if not spawn_points:
                break
            spawn_point = spawn_points.pop()
            defend = PerceptionDefender()
            vehicle = defend.build_car("defend", self.world, spawn_point)
            if vehicle is not None:
                self.cars.append(defend)
                print(f"Spawned defender{i} at {spawn_point}")
        '''

        print("Finished spawning vehicles.")

	# Run the simulation
    def run_simulation_phase(self, defense_enabled):
        self.cars = []
        self.collision_sensors = []
        # self.evaluation = Evaluation()  # Angel's code - uncomment if evaluation is needed
        
        print(f"Running simulation phase with defense_enabled={defense_enabled}...")
        self.spawn_vehicles()
        # self.attach_collision_sensors()
        
        start_time = time.time()
        while time.time() - start_time < self.run_duration:
            for car_obj in self.cars:
                #car_obj.update() 
                pass
            time.sleep(1)
        
        self.cleanup()
        print("Simulation phase completed.")
        # For evaluation, uncomment the following lines:
        # print("Evaluation logs:")
        # self.evaluation.summarize()

	# Destroy all vehicles and sensors
    def cleanup(self):
        print("Cleaning up sensors and vehicles...")
        actor_ids = []
        # Collect sensor actor IDs.
        for sensor in self.collision_sensors:
            if sensor is not None:
                actor_ids.append(sensor.id)
        # Collect vehicle actor IDs.
        for car_obj in self.cars:
            if car_obj.vehicle is not None:
                actor_ids.append(car_obj.vehicle.id)
                
        if actor_ids:
            commands = [carla.command.DestroyActor(x) for x in actor_ids]
            self.client.apply_batch_sync(commands)     
        print("Cleanup complete.")

if __name__ == "__main__":
    carla_path = r"D:\CARLA\CARLA_0.9.15\WindowsNoEditor\CarlaUE4.exe"
    port = 2000

    #start_carla(carla_path, port, cwd=carla_path)
    start_carla(carla_path, port)

    simulator = Simulator(run_duration=60)
    simulator.connect()
    
    simulator.run_simulation_phase(defense_enabled=False)
    time.sleep(10)
    simulator.run_simulation_phase(defense_enabled=True)