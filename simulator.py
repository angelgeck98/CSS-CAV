import carla
import subprocess
import random
import time
import queue
import numpy as np 
from scripts.Evaluation import DetectionEvaluator

from scripts.Car import Car
from mvp.attack.attacker import Attacker
from mvp.defense.perception_defender import PerceptionDefender

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
        # CARLA variables
        self.port = port
        self.run_duration = run_duration
        self.client = None
        self.world = None
        self.cars = []

        # Evaluation variables
        self.point_cloud_data = None 
        self.ground_truth_boxes = None 
        self.frame_id = 0 
        self.evalutator = None

    def get_point_cloud(self):
            """Get global point cloud data from all vehicles"""
            point_clouds = []
            for car_obj in self.cars: 
                if hasattr(car_obj, 'lidar'): #Make sure car has Lidar
                    point_cloud = car_obj.lidar
                    point_clouds.append(point_cloud)
            return np.vstack(point_clouds) if point_clouds else None
            
    def get_vehicle_boxes(self):
            """Get ground truth boxes for all vehicles"""
            ground_truth = []
            for car_obj in self.cars:
                if car_obj.vehicle is not None:
                    transform = car_obj.vehicle.get_transform()
                    bbox = car_obj.vehicle.bounding_box

                    gt_box = np.array([
                        transform.location.x,
                        transform.location.y,
                        transform.location.z, 
                        bbox.extent.x * 2, 
                        bbox.extent.y * 2,
                        bbox.extent.z * 2,
                        transform.rotation.yaw
                    ])

                    ground_truth.append(gt_box)
            return np.array(ground_truth)
    
	# Initialize the CARLA client and world
    def connect(self):
        self.client = carla.Client('localhost', self.port)
        print(f"Connecting to CARLA server at localhost:{self.port}...")
        self.client.set_timeout(60.0)
        self.world = self.client.get_world()
        print("Successfully connected to CARLA server.")

	# Spawn vehicles into the simulation
    def spawn_vehicles(self):
        print("Spawning vehicles...")
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        '''
        # Spawn in basic Cars
        for i in range(1, 6):
            if not spawn_points:
                break
            spawn_point = spawn_points.pop()
            car = Car(lidar_queue)
            vehicle = car.build_car("defend", self.world, spawn_point)
            if vehicle is not None:
                self.cars.append(car)
                print(f"Spawned car{i} at {spawn_point}")
        '''

        # Spawn in Attackers
        for i in range(1, 6):
            if not spawn_points:
                break
            spawn_point = spawn_points.pop()
            attack = Attacker(lidar_queue)
            vehicle = attack.build_car("attack", self.world, spawn_point)
            if vehicle is not None:
                self.cars.append(attack)
                print(f"Spawned attacker{i} at {spawn_point}")

        # Spawn in Defenders
        for i in range(1, 16):
            if not spawn_points:
                break
            spawn_point = spawn_points.pop()
            defend = PerceptionDefender()
            vehicle = defend.build_car("defend", self.world, spawn_point)
            if vehicle is not None:
                self.cars.append(defend)
                print(f"Spawned defender{i} at {spawn_point}")
        
        print("Finished spawning vehicles.")

    # Add evaluator to simulator
    def set_evaluator(self, evaluator):
        self.evaluator = evaluator

	# Run the simulation
    def run_simulation_phase(self, defense_enabled):
        self.cars = []
        self.collision_sensors = []
        # self.evaluation = Evaluation()  # Angel's code - uncomment if evaluation is needed
        self.frame_id = 0
        
        print(f"Running simulation phase with defense_enabled={defense_enabled}...")
        self.spawn_vehicles()
        # self.attach_collision_sensors()
        
        start_time = time.time()
        while time.time() - start_time < self.run_duration:
            if defense_enabled:
                for car in self.cars:
                    car.fuse_peer_scans(self.world)
                    #Run evaluation if set
                    if self.evaluator is not None: 
                        self.evaluator.evaluate_frame(
                            simulator=self, 
                            car_detector=car, # Pass the last car_obj or specific detector
                            frame_id = self.frame_id
                        )
                    if isinstance(car, PerceptionDefender):
                        car.send_v2x_message() 

                    self.frame_id += 1 # Increment Frame counter  
                    pass
            else:
                for car in self.cars:
                    car.fuse_peer_scans(self.world)
                    #Run evaluation if set
                    if self.evaluator is not None: 
                        self.evaluator.evaluate_frame(
                            simulator=self, 
                            car_detector=car, # Pass the last car_obj or specific detector
                            frame_id = self.frame_id
                        )
                    Car.send_v2x_message(car)

                    self.frame_id += 1 # Increment Frame counter  
                    pass
            time.sleep(0.1)
        
        self.cleanup()
        print("Simulation phase completed.")

        # Output evaluation
        print("Evaluation logs:")
        self.evaluation.summarize()

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
    lidar_queue = queue.Queue()

    start_carla(carla_path, port)

    simulator = Simulator(run_duration=60)
    evaluator = DetectionEvaluator()

    simulator.set_evaluator(evaluator)
    simulator.connect()
    
    # Phase 1 Evaluation - run without firewall
    simulator.run_simulation_phase(defense_enabled=False)
    print("\nPhase 1 Evaluation Results:")
    print(evaluator.calculate_final_metrics())
    evaluator.visualize_results()

    time.sleep(10)

    # Phase 2 Evaluation - run with firewall
    simulator.run_simulation_phase(defense_enabled=True)
    print("\nPhase 2 Evaluation Results:")
    print(evaluator.calculate_final_metrics())
    evaluator.visualize_results()