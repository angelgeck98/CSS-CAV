import carla
import random
import numpy as np
class Car():
    def __init__(self, lidar_queue):
        self.name = "base"
        self.affinity_score = 1.0 
        self.vehicle = None
        self.lidar = None
        self.own_scan = None
        self.collab_scan = None
        # receive the shared queue
        self.lidar_queue = lidar_queue

    # Updates affinity score by taking average of old and new score to determine trustworthiness
    def affinity_score_update(self, score):
        self.score_list = np.append(self.score_list, score)
        temp_score = self.affinity_score + score
        self.affinity_score = temp_score / 2.0
    
    # Spawns in a CARLA model of a car
    def build_car(self, behavior, world, spawn_point):
        if self.vehicle is not None:
            print("Car already built!")
            return self.vehicle

        blueprint_library = world.get_blueprint_library()

        # Changes how attackers and defenders look
        if behavior == "defend":
            vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
            available_colors = vehicle_bp.get_attribute('color').recommended_values
            vehicle_bp.set_attribute('color', random.choice(available_colors))
        elif behavior == "attack":
            vehicle_bp = blueprint_library.find('vehicle.tesla.cybertruck')

        self.vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        
        # Fixes AttributeError when trying to add sensors to Cars
        if self.vehicle is None:
            print("Spawn failed for behavior:", behavior)
            return None

        self.vehicle.set_autopilot(True)
        
        self.attach_lidar(world)
        
        return self.vehicle
    
    # Attaches LiDAR sensors to the Car
    def attach_lidar(self, world):
        bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        bp.set_attribute('channels','32')
        bp.set_attribute('points_per_second','56000')
        bp.set_attribute('rotation_frequency','20')
        bp.set_attribute('sensor_tick','0.1')
        self.lidar = world.spawn_actor(
            bp,
            carla.Transform(carla.Location(z=2.5)),
            attach_to=self.vehicle
        )
        self.lidar.listen(self.on_lidar)
    
    # Callback function for lidar data, converts the raw data to a numpy array and 
    # then puts it into the lidar queue for processing 
    # https://carla.readthedocs.io/en/0.9.15/ref_sensors/#lidar-sensor
    def on_lidar(self, data):
        points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1,4)
        self.lidar_queue.put((self.vehicle.id, data.frame, points))
        self.own_scan = points

    # Fuses the lidar data from the peer vehicles with the own vehicle's lidar data
    # https://numpy.org/doc/stable/reference/generated/numpy.linalg.inv.html
    def fuse_peer_scans(self, world):
        if self.own_scan is None:
            return
        scans = [self.own_scan]
        # Was alot simpler, but had to use numpy for LiDAR 
        while not self.lidar_queue.empty():
            vid, frame, pts = self.lidar_queue.get()
            if vid == self.vehicle.id:
                continue
            peer_transform = world.get_actor(vid).get_transform()
            self_transform  = self.vehicle.get_transform()
            peer_matrix = np.array(peer_transform.get_matrix())
            self_matrix  = np.array(self_transform.get_matrix())
            homogeneous_coords = np.hstack((pts[:, :3], np.ones((pts.shape[0],1))))
            world_pts = (peer_matrix @ homogeneous_coords.T).T
            ego_pts   = (np.linalg.inv(self_matrix) @ world_pts.T).T[:, :3]
            fused = np.hstack((ego_pts, pts[:,3:4]))
            scans.append(fused)
        self.collab_scan = np.vstack(scans)