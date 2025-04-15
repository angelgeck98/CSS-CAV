import carla
import random

class Car():
    def __init__(self):
        self.name = "base"
        self.affinity_score = 0
        self.behavior = None # what is behavior supposed to be?
        self.vehicle = None

    def affinity_score_update(self, score):
        # update this to have actual algorithm 
        self.affinity_score = score
    
    def build_car(self, world):
        if self.vehicle is not None:
            print("Car already built!")
            return

        blueprint_library = world.get_blueprint_library()
        spawn_points = world.get_map().get_spawn_points()

        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        available_colors = vehicle_bp.get_attribute('color').recommended_values
        vehicle_bp.set_attribute('color', random.choice(available_colors))

        spawn_point = random.choice(spawn_points)
        self.vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        self.vehicle.set_autopilot(True) # can change this for our demo

