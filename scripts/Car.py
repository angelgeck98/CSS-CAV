# import carla
# import random
# import numpy as np

# from mvp.attack.lidar_spoof_early_attacker import LidarSpoofEarlyAttacker

# class Car():
#     def __init__(self):
#         self.name = "base"
#         self.affinity_score = 1.0 
#         # self.behavior = None     # is this needed? 
#         self.vehicle = None

#     def affinity_score_update(self, score):
#         # affinity score is updated based on taking the average of the old and new score 
#         # to determine a "percentage" of trustworthiness
#         temp_score = self.affinity_score + score
#         self.affinity_score = temp_score / 2.0
    
#     def build_car(self, behavior, world, spawn_point):
#         self.name = behavior 
#         if self.vehicle is not None:
#             print("Car already built!")
#             return self.vehicle

#         blueprint_library = world.get_blueprint_library()

#         if behavior == "defend":
#             vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
#             available_colors = vehicle_bp.get_attribute('color').recommended_values
#             vehicle_bp.set_attribute('color', random.choice(available_colors))
#         elif behavior == "attack":
#             vehicle_bp = blueprint_library.find('vehicle.tesla.cybertruck')

#         self.vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        
#         # Hopefully fixes AttributeError: 'NoneType' object has no attribute 'set_autopilot'
#         if self.vehicle is None:
#             print("Spawn failed for behavior:", behavior)
#             return None

#         self.vehicle.set_autopilot(True)
#         return self.vehicle
#     def attach_lidar(self, world):
#         bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
#         bp.set_attribute('range', '50.0')
#         bp.set_attribute('rotation_frequency', '10')
#         bp.set_attribute('channels', '32')
#         bp.set_attribute('points_per_second', '56000')

#         lidar_transform = carla.Transform(carla.Location(x=0, z=2.5))
#         self.lidar_sensor = world.spawn_actor(bp, lidar_transform, attach_to=self.vehicle)

#     def lidar_callback(data):
#         points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
    
#         if self.name == "attack" and self.spoofer is not None:
#             spoofed = self.spoofer.spoof(self.vehicle, points)
#             print(f"[ATTACKER {self.vehicle.id}] Points with spoof: {spoofed.shape[0]}")
#         else:
#             print(f"[DEFENDER {self.vehicle.id}] Points: {points.shape[0]}")
        
#         # 🚨 Example spoof detection logic (tune this however you want)
#         if points.shape[0] > 16500:
#             self.vehicle.set_autopilot(False)
#             self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
#             print(f"[DEFENDER {self.vehicle.id}] 🛑 Braking due to suspected spoofing!")

#         self.lidar_sensor.listen(lidar_callback)
import carla
import random
import numpy as np  # Needed for LiDAR callback

from mvp.attack.lidar_spoof_early_attacker import LidarSpoofEarlyAttacker

class Car():
    def __init__(self):
        self.name = "base"
        self.affinity_score = 1.0 
        self.vehicle = None
        self.spoofer = None  # optional: init here for clarity

    def affinity_score_update(self, score):
        temp_score = self.affinity_score + score
        self.affinity_score = temp_score / 2.0

    def build_car(self, behavior, world, spawn_point):
        self.name = behavior  # Assign role name

        if self.vehicle is not None:
            print("Car already built!")
            return self.vehicle

        blueprint_library = world.get_blueprint_library()

        if behavior == "defend":
            vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
            available_colors = vehicle_bp.get_attribute('color').recommended_values
            vehicle_bp.set_attribute('color', random.choice(available_colors))
        elif behavior == "attack":
            vehicle_bp = blueprint_library.find('vehicle.tesla.cybertruck')
            self.spoofer = LidarSpoofEarlyAttacker()  # Assign spoofing tool to attackers only

        self.vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

        if self.vehicle is None:
            print("Spawn failed for behavior:", behavior)
            return None

        self.vehicle.set_autopilot(True)
        return self.vehicle

    def attach_lidar(self, world):
        bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        bp.set_attribute('range', '50.0')
        bp.set_attribute('rotation_frequency', '10')
        bp.set_attribute('channels', '32')
        bp.set_attribute('points_per_second', '56000')

        lidar_transform = carla.Transform(carla.Location(x=0, z=2.5))
        self.lidar_sensor = world.spawn_actor(bp, lidar_transform, attach_to=self.vehicle)

        def lidar_callback(data):
            points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)

            print(f"[{self.name.upper()}] {self.vehicle.id} — {points.shape[0]} pts")

            if self.name == "attack" and self.spoofer is not None:
                spoofed = self.spoofer.spoof(self.vehicle, points)
                print(f"[ATTACKER {self.vehicle.id}] Points with spoof: {spoofed.shape[0]}")
            else:
                print(f"[DEFENDER {self.vehicle.id}] Points: {points.shape[0]}")

                # 🚨 Example spoof detection
                if points.shape[0] > 16500:
                    self.vehicle.set_autopilot(False)
                    self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
                    print(f"[DEFENDER {self.vehicle.id}] 🛑 Braking due to suspected spoofing!")

        self.lidar_sensor.listen(lidar_callback)
