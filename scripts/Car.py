import carla
import random
import numpy as np  

from mvp.attack.lidar_spoof_early_attacker import LidarSpoofEarlyAttacker

class Car():
    def __init__(self):
        self.name = "base"
        self.affinity_score = 1.0 
        self.vehicle = None
        self.spoofer = None 
        self.already_spoofed = False
        self.frame_counter = 0
        self.attack_every_n_frames = 10 

    def affinity_score_update(self, score):
        temp_score = self.affinity_score + score
        self.affinity_score = temp_score / 2.0

    def build_car(self, behavior, world, spawn_point):
        self.name = behavior  

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
            self.spoofer = LidarSpoofEarlyAttacker() 

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
                self.frame_counter += 1

                if not self.already_spoofed and self.frame_counter % self.attack_every_n_frames == 0:
                    spoofed = self.spoofer.spoof(self.vehicle, points)
                    print(f"[ATTACKER {self.vehicle.id}] Spoofed! {spoofed.shape[0]} points at frame {data.frame}")
                    self.already_spoofed = True
            else:
                print(f"[DEFENDER {self.vehicle.id}] Points: {points.shape[0]}")

                # 🚨 Example spoof detection
                if points.shape[0] > 16500:
                    self.vehicle.set_autopilot(False)
                    self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
                    print(f"[DEFENDER {self.vehicle.id}] Braking due to suspected spoofing!!")

        self.lidar_sensor.listen(lidar_callback)
