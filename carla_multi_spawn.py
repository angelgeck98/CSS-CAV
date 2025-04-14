import carla
import random
import time
import atexit

spawned_vehicles = []  # global list
persistent_actors = []  # prevent garbage collection


def cleanup():
    print("\nShutting down simulation...")
    for vehicle in spawned_vehicles:
        if vehicle.is_alive:
            vehicle.destroy()
    print("✅ All spawned vehicles destroyed.")

atexit.register(cleanup)

def main():
    try:
        client = carla.Client("172.30.160.1", 2000)
        client.set_timeout(20.0)

        # 🔁 RESET the CARLA world before anything else
        world = client.get_world()
        world = client.reload_world()
        time.sleep(2)  # Optional delay to allow reset to take effect

        blueprint_library = world.get_blueprint_library()
        vehicle_blueprints = blueprint_library.filter("vehicle.*")
        tesla_blueprints = blueprint_library.filter("vehicle.tesla.*")

        spawn_points = world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        tesla_vehicle = None

        # Ensure at least one Tesla is spawned first
        for point in spawn_points:
            if tesla_vehicle:
                break
            for tesla_bp in tesla_blueprints:
                tesla_vehicle = world.try_spawn_actor(tesla_bp, point)
                if tesla_vehicle:
                    tesla_vehicle.set_autopilot(True)
                    spawned_vehicles.append(tesla_vehicle)
                    persistent_actors.append(tesla_vehicle)
                    print(f"🚗 Spawned Tesla: {tesla_vehicle.type_id} at {point.location}")
                    break

        if not tesla_vehicle:
            print("❌ Could not spawn a Tesla vehicle. Aborting.")
            return

        # Spawn other vehicles, avoiding additional Teslas
        non_tesla_blueprints = [bp for bp in vehicle_blueprints if "tesla" not in bp.id.lower()]
        remaining_points = [pt for pt in spawn_points if pt.location != tesla_vehicle.get_transform().location]
        num_to_spawn = min(14, len(remaining_points))

        for i in range(num_to_spawn):
            blueprint = random.choice(non_tesla_blueprints)
            transform = remaining_points[i]
            vehicle = world.try_spawn_actor(blueprint, transform)
            if vehicle:
                vehicle.set_autopilot(True)
                spawned_vehicles.append(vehicle)

        print("Simulation running. Type 'spoof' to inject a spoofed vehicle or 'remove' to hide an object from Tesla's view.\n")
        while True:
            cmd = input("Enter command ('spoof' or 'remove'): ").strip().lower()

            if cmd == "spoof":
                # Confirm Tesla is still present
                all_actors = world.get_actors().filter("vehicle.tesla.*")
                tesla_vehicle = next((v for v in all_actors if v.id == tesla_vehicle.id), None)
                if not tesla_vehicle:
                    print("⚠️ Tesla no longer exists in the world. Cannot spoof.")
                    continue

                spoof_blueprint = random.choice(non_tesla_blueprints)
                tesla_transform = tesla_vehicle.get_transform()
                tesla_location = tesla_transform.location
                forward_vector = tesla_transform.get_forward_vector()

                for distance_ahead in [8, 6, 4, 2]:
                    spoof_location = carla.Location(
                        x=tesla_location.x + distance_ahead * forward_vector.x,
                        y=tesla_location.y + distance_ahead * forward_vector.y,
                        z=tesla_location.z + 0.5
                    )
                    spoof_transform = carla.Transform(spoof_location, tesla_transform.rotation)

                    spoofed_vehicle = world.try_spawn_actor(spoof_blueprint, spoof_transform)
                    if spoofed_vehicle:
                        spoofed_vehicle.set_autopilot(False)
                        spoofed_vehicle.set_simulate_physics(False)  # Makes it stay still
                        persistent_actors.append(spoofed_vehicle)
                        spawned_vehicles.append(spoofed_vehicle)
                        print(f"✅ Spoofed vehicle {spoofed_vehicle.type_id} in front of Tesla at {spoof_location}")
                        break
                else:
                    print(f"❌ Could not spoof in front of Tesla {tesla_vehicle.type_id}")

            elif cmd == "remove":
                if not tesla_vehicle or not tesla_vehicle.is_alive:
                    print("⚠️ Tesla is no longer available. Cannot remove nearby objects.")
                    continue

                tesla_loc = tesla_vehicle.get_transform().location
                all_vehicles = world.get_actors().filter("vehicle.*")
                closest_vehicle = None
                min_dist = float('inf')

                for vehicle in all_vehicles:
                    if vehicle.id != tesla_vehicle.id and vehicle.is_alive:
                        dist = vehicle.get_transform().location.distance(tesla_loc)
                        if dist < min_dist and dist < 20:
                            closest_vehicle = vehicle
                            min_dist = dist

                if closest_vehicle:
                    print(f"🚫 Removing {closest_vehicle.type_id} at distance {min_dist:.2f}m")
                    try:
                        closest_vehicle.destroy()
                        if closest_vehicle in spawned_vehicles:
                            spawned_vehicles.remove(closest_vehicle)
                    except Exception as e:
                        print(f"⚠️ Failed to remove vehicle: {e}")
                else:
                    print("⚠️ No removable vehicle near Tesla.")

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
