import numpy as np
import sys
import os

# Add the project root to Python path so we can import mvp
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.Evaluation import DetectionEvaluator

class MockSimulator:
    """Mock simulator class to provide ground truth data"""
    def __init__(self):
        self.point_cloud = np.random.rand(1000, 3)  # Random point cloud
        # Format: [x, y, z, length, width, height, heading_angle]
        # Note: heading_angle should be in radians
        # Note: length, width, height are in meters
        self.gt_boxes = np.array([
            [0, 0, 0, 4.0, 2.0, 1.5, 0.0],  # x, y, z, length, width, height, heading
            [10, 5, 0, 4.0, 2.0, 1.5, np.pi/4],  # 45 degrees in radians
            [20, -5, 0, 4.0, 2.0, 1.5, np.pi/2]  # 90 degrees in radians
        ])

    def get_point_cloud(self):
        return self.point_cloud

    def get_vehicle_boxes(self):
        return self.gt_boxes

class MockCarDetector:
    """Mock car detector class to provide detection results"""
    def __init__(self, perfect_detection=False):
        self.perfect_detection = perfect_detection
        self.detected_boxes = None
        self.affinity_scores = None
        self._generate_detections()

    def _generate_detections(self):
        if self.perfect_detection:
            # Perfect detection case
            self.detected_boxes = np.array([
                [0, 0, 0, 4.0, 2.0, 1.5, 0.0],
                [10, 5, 0, 4.0, 2.0, 1.5, np.pi/4],
                [20, -5, 0, 4.0, 2.0, 1.5, np.pi/2]
            ])
            self.affinity_scores = np.array([0.9, 0.95, 0.85])
        else:
            # Imperfect detection case with errors:
            # - Misses one ground truth box (false negative)
            # - Has one false positive
            # - Has slight position error in one detection
            self.detected_boxes = np.array([
                [0.5, 0.2, 0, 4.0, 2.0, 1.5, 0.1],  # Slightly incorrect detection
                [30, 10, 0, 4.0, 2.0, 1.5, 0.0],  # False positive
                [20, -5, 0, 4.0, 2.0, 1.5, np.pi/2]  # Correct detection
                # Missing the box at [10, 5, 0] - false negative
            ])
            self.affinity_scores = np.array([0.85, 0.8, 0.9])

    def get_detected_vehicles(self):
        return self.detected_boxes

    def get_affinity_scores(self):
        return self.affinity_scores

def test_perfect_detection():
    """Test evaluation with perfect detections"""
    print("\nTesting perfect detection scenario...")
    evaluator = DetectionEvaluator()
    simulator = MockSimulator()
    detector = MockCarDetector(perfect_detection=True)

    # Evaluate multiple frames
    for frame_id in range(5):
        evaluator.evaluate_frame(simulator, detector, frame_id)

    # Get and print results
    metrics = evaluator.calculate_final_metrics()
    print("Perfect Detection Metrics:")
    print(f"Precision: {metrics['precision']:.3f}")
    print(f"Recall: {metrics['recall']:.3f}")
    print(f"F1 Score: {metrics['f1_score']:.3f}")
    print(f"Average IoU: {metrics['average_iou']:.3f}")
    print(f"Average Confidence: {metrics['average_confidence']:.3f}")

    # Verify perfect detection results
    assert metrics['precision'] == 1.0, "Precision should be 1.0 for perfect detection"
    assert metrics['recall'] == 1.0, "Recall should be 1.0 for perfect detection"
    assert metrics['f1_score'] == 1.0, "F1 score should be 1.0 for perfect detection"

def test_imperfect_detection():
    """Test evaluation with imperfect detections"""
    print("\nTesting imperfect detection scenario...")
    evaluator = DetectionEvaluator()
    simulator = MockSimulator()
    detector = MockCarDetector(perfect_detection=False)

    # Evaluate multiple frames
    for frame_id in range(5):
        evaluator.evaluate_frame(simulator, detector, frame_id)

    # Get and print results
    metrics = evaluator.calculate_final_metrics()
    print("Imperfect Detection Metrics:")
    print(f"Precision: {metrics['precision']:.3f}")
    print(f"Recall: {metrics['recall']:.3f}")
    print(f"F1 Score: {metrics['f1_score']:.3f}")
    print(f"Average IoU: {metrics['average_iou']:.3f}")
    print(f"Average Confidence: {metrics['average_confidence']:.3f}")

    # Verify imperfect detection results
    assert metrics['precision'] < 1.0, "Precision should be less than 1.0 for imperfect detection"
    assert metrics['recall'] < 1.0, "Recall should be less than 1.0 for imperfect detection"
    assert metrics['f1_score'] < 1.0, "F1 score should be less than 1.0 for imperfect detection"

def test_visualization():
    """Test visualization functionality"""
    print("\nTesting visualization...")
    evaluator = DetectionEvaluator()
    simulator = MockSimulator()
    detector = MockCarDetector(perfect_detection=False)

    # Evaluate one frame
    evaluator.evaluate_frame(simulator, detector, 0)

    # Try to visualize results
    try:
        # Visualize with just the frame_id
        evaluator.visualize_results(frame_id=0)
        print("Visualization completed successfully")
    except Exception as e:
        print(f"Visualization failed: {e}")

if __name__ == "__main__":
    print("Starting evaluation tests...")
    
    # Run tests
    test_perfect_detection()
    test_imperfect_detection()
    test_visualization()
    
    print("\nAll tests completed!") 