import carla
import subprocess
import random
import time
# from car import Car                # Kayla's code
# from evaluation import Evaluation  # Angel's code
# assuming Car uses attacker/defender code from Yaz's code

# Starts the CARLA simulator 
def start_carla(carla_path, port, cwd):
    subprocess.Popen(['CarlaUE4.exe', '-quality-level=Low', f'-carla-port={port}'], cwd=cwd)
    print("Starting CARLA simulator...")
    time.sleep(5)

class Simulator:
    def __init__(self, host='localhost', port=2000, run_duration=60):
        self.host = host
        self.port = port
        self.run_duration = run_duration
        self.client = None
        self.world = None
        self.cars = []
        # self.evaluation = Evaluation()  # Angel's code, may or may not be needed
        self.collision_sensors = []

	# Initialize the CARLA client and world
    def connect(self):
        self.client = carla.Client(self.host, self.port)
        print(f"Connecting to CARLA server at {self.host}:{self.port}...")
        self.client.set_timeout(60.0)
        self.world = self.client.get_world()
        print("Successfully connected to CARLA server.")

	# Spawn vehicles in the simulation
    def spawn_vehicles(self):
        print("Spawning vehicles...")
        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprints = list(blueprint_library.filter('vehicle.*'))
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        
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
        print("Finished spawning vehicles.")

	# Attach collision sensors to vehicles
    # NOTE: Only attaches collision sensors, either rewrite or add more methods
    # https://carla.readthedocs.io/en/latest/core_sensors/
    def attach_collision_sensors(self):
        print("Attaching collision sensors to vehicles...")
        blueprint_library = self.world.get_blueprint_library()
        collision_sensor_bp = blueprint_library.find('sensor.other.collision')
        
        for car_obj in self.cars:
            sensor_transform = carla.Transform(carla.Location(x=0.0, z=2.0))
            sensor = self.world.spawn_actor(collision_sensor_bp, sensor_transform, attach_to=car_obj.vehicle)
            # Use a lambda to capture the vehicle ID alongside collision event
            sensor.listen(lambda event, cid=car_obj.vehicle.id: self.log_collision(event, cid))
            car_obj.attach_sensor("collision", sensor)
            self.collision_sensors.append(sensor)
        print("Collision sensors attached.")

	# Used by collision sensors to log collisions
	# @ANGEL: lmk if you want this for your evaluation class
    def log_collision(self, event, car_id):
        other_actor = event.other_actor
        impulse = event.normal_impulse
        log_message = f"Vehicle {car_id} collided with {other_actor.type_id} (impulse: {impulse})"
        print(log_message)
        self.evaluation.record_event(log_message)

	# Run the simulation
    def run_simulation_phase(self, defense_enabled):
        self.cars = []
        self.collision_sensors = []
        # self.evaluation = Evaluation()  # Angel's code - uncomment if evaluation is needed
        
        print(f"Running simulation phase with defense_enabled={defense_enabled}...")
        self.spawn_vehicles(defense_enabled)
        self.attach_collision_sensors()
        
        start_time = time.time()
        while time.time() - start_time < self.run_duration:
            for car_obj in self.cars:
                car_obj.update()
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
    carla_path = 'E:/DEV/CSS-CAV'
    port = 2000

    start_carla(carla_path, port, cwd=carla_path)

    simulator = Simulator(run_duration=60)
    simulator.connect()
    
    simulator.run_simulation_phase(defense_enabled=False)
    time.sleep(10)
    simulator.run_simulation_phase(defense_enabled=True)