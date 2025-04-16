import carla
import random
import json
import time

class Car():
    def __init__(self):
        self.name = "base"
        self.affinity_score = 1.0 
        self.vehicle = None
        self.v2x_sensor = None
        self.received_v2x_messages = []

    def affinity_score_update(self, score):
        # affinity score is updated based on taking the average of the old and new score 
        # to determine a "percentage" of trustworthiness
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
        
        # Initialize V2X sensor for CAV communication
        self.init_v2x_sensor(world)
        
        return self.vehicle
    
    # Initialize V2X sensor
    # https://carla.readthedocs.io/en/latest/ref_sensors/#custom-v2x-message
    def init_v2x_sensor(self, world):
        blueprint_library = world.get_blueprint_library()
        v2x_bp = blueprint_library.find('sensor.other.v2x_custom')
        # Optionally, configure transmission and reception settings.
        v2x_bp.set_attribute('transmit_power', '21.5')
        v2x_bp.set_attribute('receiver_sensitivity', '-99')
        v2x_bp.set_attribute('frequency_ghz', '5.9')
        # Attach the V2X sensor on the vehicle’s roof.
        sensor_location = carla.Location(x=0.0, y=0.0, z=2.5)
        sensor_rotation = carla.Rotation()
        sensor_transform = carla.Transform(sensor_location, sensor_rotation)
        self.v2x_sensor = world.spawn_actor(v2x_bp, sensor_transform, attach_to=self.vehicle)
        # Register a callback for incoming messages.
        self.v2x_sensor.listen(lambda sensor_data: self._v2x_callback(sensor_data))
        print(f"V2X sensor attached to vehicle ID {self.vehicle.id}")

    # Used by the V2X sensor to receive messages from other vehicles.
    def _v2x_callback(self, sensor_data):
        # Process the received sensor data; assume sensor_data.data is sent as a bytes object.
        try:
            msg_str = sensor_data.data.decode('utf-8')
            message = json.loads(msg_str)
        except Exception as e:
            print("Error getting V2X message:", e)
            return
        # Ignore messages sent by self.
        if "id" in message and message["id"] == self.vehicle.id:
            return
        self.received_v2x_messages.append(message)
        print(f"Vehicle {self.vehicle.id} received message from Vehicle {message.get('id')}: {message}")

    # Send a V2X message containing vehicle state information.
    def send_v2x_message(self):
        if self.v2x_sensor is None:
            print("V2X sensor not initialized!")
            return
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        message = {
            "id": self.vehicle.id,
            "transform": {
                "location": {
                    "x": transform.location.x,
                    "y": transform.location.y,
                    "z": transform.location.z
                },
                "rotation": {
                    "pitch": transform.rotation.pitch,
                    "yaw": transform.rotation.yaw,
                    "roll": transform.rotation.roll
                }
            },
            "velocity": {
                "x": velocity.x,
                "y": velocity.y,
                "z": velocity.z
            },
            "timestamp": time.time()
        }
        msg_str = json.dumps(message)
        # Use the sensor's send() method to transmit the message.
        self.v2x_sensor.send(msg_str)
        print(f"Vehicle {self.vehicle.id} sent V2X message: {msg_str}")

    #  Fuse received V2X messages to create a collaborative state.
    def fuse_collaborative_data(self):
        # An example fusion: compute the average position from received messages.
        if not self.received_v2x_messages:
            return None
        sum_x = sum(msg["transform"]["location"]["x"] for msg in self.received_v2x_messages if "transform" in msg)
        sum_y = sum(msg["transform"]["location"]["y"] for msg in self.received_v2x_messages if "transform" in msg)
        count = len([msg for msg in self.received_v2x_messages if "transform" in msg])
        if count == 0:
            return None
        avg_x = sum_x / count
        avg_y = sum_y / count
        # Clear messages after fusion.
        self.received_v2x_messages.clear()
        fused_data = {"avg_collab_position": {"x": avg_x, "y": avg_y}}
        print(f"Vehicle {self.vehicle.id} fused collaborative data: {fused_data}")
        return fused_data
