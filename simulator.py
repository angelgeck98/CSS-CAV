# import carla
# import subprocess
# import random
# import time

# from scripts.Car import Car          # Kayla's code
# #from mvp.attack.attacker import Attacker
# from scripts.Car import Car
# from mvp.defense.perception_defender import PerceptionDefender

# # from evaluation import Evaluation  # Angel's code
# # assuming Car uses attacker/defender code from Yaz's code

# # Starts the CARLA simulator 

# # changed by Kayla so this works on my PC, get rid of stuff between meows to go back to normal
# # also change how this is called in main
# #def start_carla(carla_path, port, cwd):
#     #subprocess.Popen(['CarlaUE4.exe', '-quality-level=Low', f'-carla-port={port}'], cwd=cwd)

#     # meow
# def start_carla(carla_path, port):
#     subprocess.Popen([carla_path, '-quality-level=Low', f'-carla-port={port}'])
#     # meow
#     print("Starting CARLA simulator...")
#     time.sleep(10)

# class Simulator:
#     def __init__(self, port=2000, run_duration=60):
#         self.port = port
#         self.run_duration = run_duration
#         self.client = None
#         self.world = None
#         self.cars = []
#         # self.evaluation = Evaluation()  # Angel's code, may or may not be needed
#         self.collision_sensors = []
#         self.attacker_vehicle = None

# 	# Initialize the CARLA client and world
#     def connect(self):
#         self.client = carla.Client('localhost', self.port)
#         print(f"Connecting to CARLA server at localhost:{self.port}...")
#         self.client.set_timeout(60.0)
#         self.world = self.client.get_world()
#         print("Successfully connected to CARLA server.")

# 	# Spawn vehicles in the simulation
#     # def spawn_vehicles(self):
#     #     print("Spawning vehicles...")
#     #     spawn_points = self.world.get_map().get_spawn_points()
#     #     random.shuffle(spawn_points)

#     #     '''
#     #     # Test basic Car spawn
#     #     for i in range(0, 25):
#     #         if not spawn_points:
#     #             break
#     #         spawn_point = spawn_points.pop()
#     #         car = Car()
#     #         vehicle = car.build_car("defend", self.world, spawn_point)
#     #         if vehicle is not None:
#     #             self.cars.append(car)
#     #             print(f"Spawned car{i} at {spawn_point}")
#     #     '''
#     #     for i in range(0, 25):
#     #         if not spawn_points:
#     #             break
#     #         spawn_point = spawn_points.pop()
#     #         #attack = Attacker()
#     #         attack = Car()
#     #         vehicle = attack.build_car("attack", self.world, spawn_point)
#     #         if vehicle is not None:
#     #             self.cars.append(attack)
#     #             if self.attacker_vehicle is None:
#     #                 self.attacker_vehicle = vehicle  # Save for spoofing
#     #             print(f"Spawned attacker{i} at {spawn_point}")

#     #     print("Finished spawning vehicles.")

#     def spawn_spoofed_vehicle(self):
#         if not self.attacker_vehicle:
#             print(" No attacker vehicle to spoof from.")
#             return

#         blueprint_library = self.world.get_blueprint_library()
#         spoof_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
#         transform = self.attacker_vehicle.get_transform()
#         forward_vector = transform.get_forward_vector()

#         spoof_loc = carla.Location(
#             x=transform.location.x + forward_vector.x * 4,
#             y=transform.location.y + forward_vector.y * 4,
#             z=transform.location.z
#         )
#         spoof_rotation = carla.Rotation(pitch=0.0, yaw=transform.rotation.yaw, roll=0.0)
#         spoof_transform = carla.Transform(spoof_loc, spoof_rotation)

#         spoofed_ped = self.world.try_spawn_actor(spoof_bp, spoof_transform)
#         if spoofed_ped:
#             print(f"poofed pedestrian successfully at {spoof_loc}")
#             self.cars.append(spoofed_ped)
#         else:
#             print(" Failed to spawn spoofed pedestrian.")

#         # Test attack spawn
#         # for i in range(0, 25):
#         #     if not spawn_points:
#         #         break
#         #     spawn_point = spawn_points.pop()
#         #     attack = Attacker()
#         #     vehicle = attack.build_car("attack", self.world, spawn_point)
#         #     if vehicle is not None:
#         #         self.cars.append(attack)
#         #         print(f"Spawned attacker{i} at {spawn_point}")

#         '''
#         # Test defense spawn
#         # doesn't work because of load_map() in perception_defender (probably bc of my path?)
#         spawn_point = spawn_points.pop()
#         defend1 = PerceptionDefender()
#         defend1.build_car("defend", self.world, spawn_point)
#         self.cars.append(defend1)
#         print(f"Spawned defender at {spawn_point}")
#         '''
        

#         '''
#         attacker_chance = 0.3
#         for bp in vehicle_blueprints:
#             if not spawn_points:
#                 break
#             spawn_point = spawn_points.pop()
#             actor = self.world.try_spawn_actor(bp, spawn_point)
# 			# https://carla.readthedocs.io/en/latest/core_actors/
#             if actor is not None:
#                 is_attacker = random.random() < attacker_chance
#                 # How I would imagine we would use the Car class, will change for Kayla's code
# 				# car_obj = Car(
#                 #     vehicle_actor=actor,
#                 #     is_attacker=is_attacker,
#                 #     attack_strength=0.7,
#                 #     initial_affinity=1.0
#                 # )
#                 # self.cars.append(car_obj)
#                 print(f"Spawned vehicle {actor.id} at {spawn_point} | Attacker: {is_attacker}")
#         '''
#         print("Finished spawning vehicles.")

# 	# Run the simulation
#     def run_simulation_phase(self, defense_enabled):
#         self.cars = []
#         self.collision_sensors = []
#         # self.evaluation = Evaluation()  # Angel's code - uncomment if evaluation is needed
#         print(f"Running simulation phase with defense_enabled={defense_enabled}...")
#         #self.spawn_vehicles()
#         self.spawn_spoofed_vehicle()

#         # self.attach_collision_sensors()
        
#         start_time = time.time()
#         while time.time() - start_time < self.run_duration:
#             for car_obj in self.cars:
#                 #car_obj.update() 
#                 pass
#             time.sleep(1)
        
#         self.cleanup()
#         print("Simulation phase completed.")
#         # For evaluation, uncomment the following lines:
#         # print("Evaluation logs:")
#         # self.evaluation.summarize()

# 	# Destroy all vehicles and sensors
#     def cleanup(self):
#         print("Cleaning up sensors and vehicles...")
#         actor_ids = []
#         # Collect sensor actor IDs.
#         for sensor in self.collision_sensors:
#             if sensor is not None:
#                 actor_ids.append(sensor.id)
#         # Collect vehicle actor IDs.
#         for car_obj in self.cars:
#             if car_obj.vehicle is not None:
#                 actor_ids.append(car_obj.vehicle.id)
                
#         if actor_ids:
#             commands = [carla.command.DestroyActor(x) for x in actor_ids]
#             self.client.apply_batch_sync(commands)     
#         print("Cleanup complete.")

# if __name__ == "__main__":
#     carla_path = r"D:\CARLA\CARLA_0.9.15\WindowsNoEditor\CarlaUE4.exe"
#     port = 2000

#     #start_carla(carla_path, port, cwd=carla_path)
#     start_carla(carla_path, port)

#     simulator = Simulator(run_duration=60)
#     simulator.connect()
    
#     simulator.run_simulation_phase(defense_enabled=False)
#     time.sleep(10)
#     simulator.run_simulation_phase(defense_enabled=True)

import carla
import subprocess
import random
import time

from scripts.Car import Car  # Kayla's code
from mvp.defense.perception_defender import PerceptionDefender

# Starts the CARLA simulator
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
        self.collision_sensors = []
        self.attacker_vehicle = None

    def connect(self):
        self.client = carla.Client('localhost', self.port)
        print(f"Connecting to CARLA server at localhost:{self.port}...")
        self.client.set_timeout(60.0)
        self.world = self.client.get_world()
        print("Successfully connected to CARLA server.")

    def spawn_vehicles(self):
        print("Spawning vehicles...")
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        for i in range(10):
            if not spawn_points: break
            car = Car()
            vehicle = car.build_car("defend", self.world, spawn_points.pop())
            if vehicle: 
                self.cars.append(car)
                car.attach_lidar(self.world)

        for i in range(15):
            if not spawn_points: break
            attacker = Car()
            vehicle = attacker.build_car("attack", self.world, spawn_points.pop())
            if vehicle:
                attacker.attach_lidar(self.world)
                self.cars.append(attacker)
                if self.attacker_vehicle is None:
                    self.attacker_vehicle = vehicle
        print("Finished spawning vehicles.")

    def run_simulation_phase(self, defense_enabled):
        self.cars = []
        self.collision_sensors = []
        print(f"Running simulation phase with defense_enabled={defense_enabled}...")
        self.spawn_vehicles()

        start_time = time.time()
        while time.time() - start_time < self.run_duration:
            for car_obj in self.cars:
                # Optional: car_obj.update() if needed
                pass
            time.sleep(1)

        self.cleanup()
        print("Simulation phase completed.")

    def cleanup(self):
        print("Cleaning up sensors and vehicles...")
        actor_ids = []
        for sensor in self.collision_sensors:
            if sensor is not None:
                actor_ids.append(sensor.id)
        for car_obj in self.cars:
            if car_obj.vehicle is not None:
                actor_ids.append(car_obj.vehicle.id)

        if actor_ids:
            commands = [carla.command.DestroyActor(x) for x in actor_ids]
            self.client.apply_batch_sync(commands)
        print("Cleanup complete.")

if __name__ == "__main__":
    carla_path = 'C:/Users/lopez/Downloads/Carla/WindowsNoEditor'
    port = 2000

    #start_carla(carla_path, port)

    simulator = Simulator(run_duration=60)
    simulator.connect()

    simulator.run_simulation_phase(defense_enabled=False)
    time.sleep(10)
    simulator.run_simulation_phase(defense_enabled=True)
