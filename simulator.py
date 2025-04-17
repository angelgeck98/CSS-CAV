import carla
import subprocess
import random
import time
import numpy

from scripts.Car import Car          # Kayla's code
from mvp.attack.attacker import Attacker
from mvp.defense.perception_defender import PerceptionDefender
# from evaluation import Evaluation  # Angel's code
# assuming Car uses attacker/defender code from Yaz's code

# Starts the CARLA simulator 

# changed by Kayla so this works on my PC, get rid of stuff between meows to go back to normal
# also change how this is called in main
#def start_carla(carla_path, port, cwd):
    #subprocess.Popen(['CarlaUE4.exe', '-quality-level=Low', f'-carla-port={port}'], cwd=cwd)

    # meow
def start_carla(carla_path, port):
    subprocess.Popen([carla_path, '-quality-level=Low', f'-carla-port={port}'])
    # meow
    print("Starting CARLA simulator...")
    time.sleep(15)

class Simulator:
    def __init__(self, port=2000, run_duration=60):
        self.port = port
        self.run_duration = run_duration
        self.client = None
        self.world = None
        self.cars = []
        # self.evaluation = Evaluation()  # Angel's code, may or may not be needed
        self.collision_sensors = []

        ###ANGEL EVALUATION DATA STUFF####
        self.point_cloud_data = None 
        self.ground_truth_boxes = None 
        self.frame_id = 0 
        self.evaluator = None # Will store DetectionEvaluator
        ###ANGEL EVALUATION DATA STUFF####

    #Get point cloud
    def get_point_cloud(self):
        """Get global point cloud data from all vehicles"""
        point_clouds = []
        for car_obj in self.cars:
            if hasattr(car_obj, 'lidar_sensor'): #MAKE sure car has LiDAR
                point_cloud = car_obj.lidar_sensor.get_data()
                point_clouds.append(point_cloud)
        return np.vstack(point_clouds) if point_clouds else None

	# Initialize the CARLA client and world
    def connect(self):
        self.client = carla.Client('localhost', self.port)
        print(f"Connecting to CARLA server at localhost:{self.port}...")
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        print("Successfully connected to CARLA server.")

	# Spawn vehicles in the simulation
    def spawn_vehicles(self):
        print("Spawning vehicles...")
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        '''
        # Test basic Car spawn
        for i in range(0, 25):
            if not spawn_points:
                break
            spawn_point = spawn_points.pop()
            car = Car()
            vehicle = car.build_car("defend", self.world, spawn_point)
            if vehicle is not None:
                self.cars.append(car)
                print(f"Spawned car{i} at {spawn_point}")
        '''
        # Test attack spawn
        for i in range(0, 25):
            if not spawn_points:
                break
            spawn_point = spawn_points.pop()
            attack = Attacker()
            vehicle = attack.build_car("attack", self.world, spawn_point)
            if vehicle is not None:
                self.cars.append(attack)
                print(f"Spawned attacker{i} at {spawn_point}")

        '''
        # Test defense spawn
        # doesn't work because of load_map() in perception_defender (probably bc of my path?)
        spawn_point = spawn_points.pop()
        defend1 = PerceptionDefender()
        defend1.build_car("defend", self.world, spawn_point)
        self.cars.append(defend1)
        print(f"Spawned defender at {spawn_point}")
        '''
        

        '''
        attacker_chance = 0.3
        for bp in vehicle_blueprints:
            if not spawn_points:
                break
            spawn_point = spawn_points.pop()
            actor = self.world.try_spawn_actor(bp, spawn_point)
			# https://carla.readthedocs.io/en/latest/core_actors/
            if actor is not None:
                is_attacker = random.random() < attacker_chance
                # How I would imagine we would use the Car class, will change for Kayla's code
				# car_obj = Car(
                #     vehicle_actor=actor,
                #     is_attacker=is_attacker,
                #     attack_strength=0.7,
                #     initial_affinity=1.0
                # )
                # self.cars.append(car_obj)
                print(f"Spawned vehicle {actor.id} at {spawn_point} | Attacker: {is_attacker}")
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
                car_obj.send_v2x_message()
            time.sleep(0.5)
            for car_obj in self.cars:
                fused = car_obj.fuse_collaborative_data()
            time.sleep(0.5)
        
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
    carla_path = r"C:\CARLA-DEV\CarlaUE4.exe"
    port = 2000

    #start_carla(carla_path, port, cwd=carla_path)
    start_carla(carla_path, port)

    simulator = Simulator(run_duration=60)
    simulator.connect()
    
    simulator.run_simulation_phase(defense_enabled=False)
    time.sleep(10)
    simulator.run_simulation_phase(defense_enabled=True)