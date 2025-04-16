import numpy as np
from mvp.visualize.evaluate import draw_detection_roc, draw_distribution
from mvp.visualize.general import draw_matplotlib

class DetectionEvaluator:
    def __init__(self):
        # Initialize evaluation parameters
        self.iou_threshold = 0.5  # Threshold for considering a detection correct
        self.confidence_threshold = 0.7  # Threshold for accepting detections
        
        # Metrics storage
        self.metrics = {
            "TP": 0,  # True Positives
            "FP": 0,  # False Positives
            "FN": 0,  # False Negatives
            "total_gt": 0,  # Total ground truth objects
            "total_pred": 0,  # Total predictions
            "iou_scores": [],  # Store all IoU scores
            "affinity_scores": []  # Store all affinity scores
        }
        
        # Store frame results if needed
        self.frame_results = {}

    def get_simulator_data(self, simulator):
        """
        Get ground truth data from simulator
        Returns: point_cloud, ground_truth_boxes
        """
        # TODO: Implement based on simulator teammate's class
        point_cloud = simulator.get_point_cloud()
        gt_boxes = simulator.get_vehicle_boxes()
        return point_cloud, gt_boxes

    def get_detection_data(self, car_detector):
        """
        Get detection results from car detector
        Returns: detected_boxes, affinity_scores
        """
        # TODO: Implement based on car detector teammate's class
        detected_boxes = car_detector.get_detected_vehicles()
        affinity_scores = car_detector.get_affinity_scores()
        return detected_boxes, affinity_scores

    def calculate_iou(self, box1, box2):
        """
        Calculate IoU between two boxes
        box format: [x, y, z, length, width, height, heading_angle]
        """
        # TODO: Implement 3D IoU calculation
        # This will be complex - might want to use existing IoU function
        pass

    def evaluate_frame(self, simulator, car_detector, frame_id):
        """
        Evaluate one frame of detections
        """
        # Get data from both sources
        point_cloud, gt_boxes = self.get_simulator_data(simulator)
        detected_boxes, affinity_scores = self.get_detection_data(car_detector)

        # Update total counts
        self.metrics["total_gt"] += len(gt_boxes)
        self.metrics["total_pred"] += len(detected_boxes)

        # Filter low confidence detections
        valid_detections = []
        valid_scores = []
        for box, score in zip(detected_boxes, affinity_scores):
            if score >= self.confidence_threshold:
                valid_detections.append(box)
                valid_scores.append(score)

        # Compare each valid detection with ground truth
        for det_box, score in zip(valid_detections, valid_scores):
            max_iou = 0
            best_gt_match = None

            # Find best matching ground truth box
            for gt_box in gt_boxes:
                iou = self.calculate_iou(det_box, gt_box)
                if iou > max_iou:
                    max_iou = iou
                    best_gt_match = gt_box

            # Store results
            self.metrics["iou_scores"].append(max_iou)
            self.metrics["affinity_scores"].append(score)

            # Update metrics
            if max_iou >= self.iou_threshold:
                self.metrics["TP"] += 1
            else:
                self.metrics["FP"] += 1

        # Calculate false negatives (missed detections)
        self.metrics["FN"] = len(gt_boxes) - self.metrics["TP"]

        # Store frame results if needed
        self.frame_results[frame_id] = {
            "point_cloud": point_cloud,
            "gt_boxes": gt_boxes,
            "detected_boxes": detected_boxes,
            "affinity_scores": affinity_scores
        }

    def calculate_final_metrics(self):
        """
        Calculate final evaluation metrics
        """
        precision = self.metrics["TP"] / (self.metrics["TP"] + self.metrics["FP"])
        recall = self.metrics["TP"] / (self.metrics["TP"] + self.metrics["FN"])
        f1_score = 2 * (precision * recall) / (precision + recall)

        return {
            "precision": precision,
            "recall": recall,
            "f1_score": f1_score,
            "average_iou": np.mean(self.metrics["iou_scores"]),
            "average_confidence": np.mean(self.metrics["affinity_scores"])
        }

    def visualize_results(self, frame_id=None):
        """
        Visualize evaluation results
        """
        # Visualize specific frame if requested
        if frame_id is not None:
            frame_data = self.frame_results[frame_id]
            draw_matplotlib(
                frame_data["point_cloud"],
                gt_boxes=frame_data["gt_boxes"],
                pred_boxes=frame_data["detected_boxes"]
            )

        # Plot score distributions
        draw_distribution(
            [self.metrics["iou_scores"], self.metrics["affinity_scores"]],
            labels=["IoU Scores", "Confidence Scores"]
        )

        # Plot ROC curve
        draw_detection_roc(
            normal_values=self.metrics["iou_scores"],
            attack_values=self.metrics["affinity_scores"]
        )

# Example usage:
"""
evaluator = DetectionEvaluator()

# For each frame
evaluator.evaluate_frame(simulator, car_detector, frame_id=1)

# Get final results
final_metrics = evaluator.calculate_final_metrics()
print(final_metrics)

# Visualize results
evaluator.visualize_results()
"""