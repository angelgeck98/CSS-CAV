import carla
import random
import numpy as np

class Car():
    def __init__(self):
        self.name = "base"
        self.affinity_score = 1.0 
        self.vehicle = None
        self.score_list = np.array([])

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
        return self.vehicle