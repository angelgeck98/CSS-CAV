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

        #Evaluation Stuff 
        self.detected_vehicles = [] #Detected vehicle boxes 
        self.detection_scores = []  #Confidence Scores
       

    def get_detected_vehicles(self):
        """Return detected vehicle boxes for evaluation"""
        if self.vehicle is None or self.collab_scan is None:
            return np.array([])
        
        detected = []
        # Get vehicles this car can detect
        for actor in self.vehicle.get_world().get_actors():
            if actor.type_id.startswith('vehicle') and actor.id != self.vehicle.id:
                transform = actor.get_transform()
                bbox = actor.bounding_box
                
                # Check if points from collab_scan fall within the bounding box
                # Convert points to vehicle's local coordinate system
                points = self.collab_scan[:, :3]
                points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
                world_matrix = np.array(transform.get_matrix())
                local_points = (np.linalg.inv(world_matrix) @ points_homogeneous.T).T[:, :3]
                
                # Check if any points fall within the bounding box
                in_box = np.all(np.abs(local_points) <= bbox.extent, axis=1)
                if np.any(in_box):
                    # Format matching evaluator's expected format
                    box = np.array([
                        transform.location.x,
                        transform.location.y,
                        transform.location.z,
                        bbox.extent.x * 2,
                        bbox.extent.y * 2,
                        bbox.extent.z * 2,
                        transform.rotation.yaw
                    ])
                    detected.append(box)
        
        return np.array(detected)
    
    def get_affinity_scores(self):
        """Return affinity scores for evaluation"""
        detected_vehicles = self.get_detected_vehicles()
        if len(detected_vehicles) > 0:
            return np.array([self.affinity_score] * len(detected_vehicles))
        return np.array([])
    
    def process_lidar_data(self):
        """Update detections for evaluation"""
        self.detected_vehicles = self.get_detected_vehicles()
        self.detection_scores = self.get_affinity_scores()
            
    ##########Evaluation stuff##########

    def affinity_score_update(self, score):
        # affinity score is updated based on taking the average of the old and new score 
        # to determine a "percentage" of trustworthiness
        self.score_list = np.append(self.score_list, score)
        temp_score = self.affinity_score + score
        self.affinity_score = temp_score / 2.0
    
    def build_car(self, behavior, world, spawn_point):
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

        self.vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        
        # Hopefully fixes AttributeError: 'NoneType' object has no attribute 'set_autopilot'
        if self.vehicle is None:
            print("Spawn failed for behavior:", behavior)
            return None

        self.vehicle.set_autopilot(True)
        
        self.attach_lidar(world)
        
        return self.vehicle
    
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
        self.process_lidar_data()

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
        self.process_lidar_data() #Update detections 