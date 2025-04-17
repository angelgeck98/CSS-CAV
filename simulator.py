import carla
import subprocess
import random
import time

from scripts.Car import Car  # Kayla's code
from mvp.defense.perception_defender import PerceptionDefender

# Starts CARLA simulator
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
