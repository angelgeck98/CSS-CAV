import numpy as np
import open3d as o3d
import copy
from collections import OrderedDict

class MockDataset:
    """Generates synthetic LiDAR data for testing"""
    def __init__(self):
        self.cache = OrderedDict()
        self.cache_size = 10
    
    def get_case(self, idx=0, tag="single_vehicle", use_lidar=True, use_camera=False):
        # Generate a car-shaped point cloud (5m long, 2m wide)
        points = []
        # Car roof
        for x in np.linspace(-2.5, 2.5, 15):
            for y in np.linspace(-1, 1, 6):
                points.append([x, y, 1.5, 0.8])  # x,y,z,intensity
        
        # Ground plane
        for x in np.linspace(-10, 10, 20):
            for y in np.linspace(-10, 10, 20):
                points.append([x, y, 0, 0.2])
        
        return {
            "lidar": np.array(points, dtype=np.float32),
            "vehicle_id": "ego",
            "lidar_pose": np.array([0, 0, 0, 0, 0, 0])  # x,y,z,roll,pitch,yaw
        }

class MockAttacker:
    """Simulates LiDAR spoofing without real attack files"""
    def __init__(self):
        self.dense = 1
        self.sync = 0
    
    def run(self, case, attack_opts):
        original = copy.deepcopy(case)
        points = original[0]["lidar"]
        
        # Simple attack: shift points along X-axis
        intensity = attack_opts.get("intensity", 0.5)
        points[:, 0] += intensity * self.dense
        
        return [original[0]], {"modified_points": len(points)}

def visualize_comparison(original, spoofed):
    """Show original vs attacked point clouds"""
    pcd_orig = o3d.geometry.PointCloud()
    pcd_orig.points = o3d.utility.Vector3dVector(original[:,:3])
    pcd_orig.paint_uniform_color([0, 0.7, 0])  # Green
    
    pcd_spoof = o3d.geometry.PointCloud()
    pcd_spoof.points = o3d.utility.Vector3dVector(spoofed[:,:3])
    pcd_spoof.paint_uniform_color([0.7, 0, 0])  # Red
    
    # Add coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0)
    
    o3d.visualization.draw_geometries(
        [pcd_orig, pcd_spoof, coord_frame],
        window_name="LiDAR Spoofing Test (Green=Original, Red=Spoofed)",
        width=1000,
        height=800
    )

def run_test():
    try:
        # 1. Setup mock environment
        dataset = MockDataset()
        attacker = MockAttacker()
        
        # 2. Generate test case
        case = [dataset.get_case()]
        original_points = case[0]["lidar"]
        
        # 3. Run attack
        spoofed_case, attack_info = attacker.run(case, {
            "intensity": 0.5,
            "frame_ids": [0],
            "victim_vehicle_id": "ego"
        })
        
        # 4. Show results
        print("=== Attack Successful ===")
        print(f"Modified {attack_info['modified_points']} points")
        print("Sample point:")
        print(f"  Before: {original_points[10][:3]}")
        print(f"  After: {spoofed_case[0]['lidar'][10][:3]}")
        
        # 5. Visualize
        visualize_comparison(original_points, spoofed_case[0]['lidar'])
        
    except Exception as e:
        print(f"Error: {str(e)}")
        print("Ensure requirements are installed: pip install open3d numpy")

if __name__ == "__main__":
    run_test()