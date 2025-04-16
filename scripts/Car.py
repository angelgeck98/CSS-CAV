import carla
import random

class Car():
    def __init__(self):
        # update these to include the ones Jordan had in simulation.py
        # is behavior just supposed to be attacker Y/N?
        self.name = "base"
        self.affinity_score = 1.0 
        self.behavior = None 
        self.vehicle = None

    def affinity_score_update(self, score):
        # affinity score is updated based on taking the average of the old score 
        # and new score - which can be 1 or 0 - to determine a "percentage" of 
        # trustworthiness
        temp_score = self.affinity_score + score
        self.affinity_score = temp_score / 2.0
    
    def build_car(self, world):
        if self.vehicle is not None:
            print("Car already built!")
            return

        # may need to alter spawn point handling so cars don't spawn in at same point
        # make global??
        blueprint_library = world.get_blueprint_library()
        spawn_points = world.get_map().get_spawn_points()

        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        available_colors = vehicle_bp.get_attribute('color').recommended_values
        vehicle_bp.set_attribute('color', random.choice(available_colors))

        spawn_point = random.choice(spawn_points)
        self.vehicle = world.spawn_actor(vehicle_bp, spawn_point)

        # need to connect Car to the traffic manager - was having issues with it
        self.vehicle.set_autopilot(True) 
