# Evaluation.py:
# Go through both simulation logs, compare success rate of attacks
import numpy as np
from mvp.visualize.evaluate import draw_detection_roc, draw_distribution
from mvp.visualize.general import draw_matplotlib
from mvp.tools.iou import iou3d, iou2d

class DetectionEvaluator:
    def __init__(self):
       # Initialize evaluation parameters
        self.iou_threshold = 0.5  # Threshold for considering a detection correct
        self.confidence_threshold = 0.7  # Threshold for accepting detections
        
        # Metrics storage
        self.metrics = self.reset_metrics()
        self.frame_results = {}

    def reset_metrics(self):
        """Reset all metrics to initial state"""
        return {
            "TP": 0,  # True Positives
            "FP": 0,  # False Positives
            "FN": 0,  # False Negatives
            "total_gt": 0,  # Total ground truth objects
            "total_pred": 0,  # Total predictions
            "iou_scores": [],  # Store all IoU scores
            "affinity_scores": [],  # Store all affinity scores
            "frame_metrics": {}  # Store per-frame metrics
        }


    def get_simulator_data(self, simulator):

        try:
            point_cloud = simulator.get_point_cloud()
            gt_boxes = simulator.get_vehicle_boxes()
            return point_cloud, gt_boxes
        except Exception as e:
            print(f"Error getting simulator data: {e}")
            return None
          
        

    def get_detection_data(self, car_detector):
        """
        Get detection results from car detector
        Returns: detected_boxes, affinity_scores
        """
        try:
            detected_boxes = car_detector.get_detected_vehicles()
            affinity_scores = car_detector.get_affinity_scores()
            return detected_boxes, affinity_scores
        except Exception as e:
            print(f"Error getting detection data: {e}")
            return None, None


    def calculate_iou(self, box1, box2):
        """
        Calculate IoU between two boxes
        box format: [x, y, z, length, width, height, heading_angle]
        Returns IoU score between 0 and 1
        """
        def get_box_bounds(box):
            x, y, z = box[0], box[1], box[2]
            l, w, h = box[3], box[4], box[5]
            # Get box bounds without considering rotation for simplicity
            return {
                'x_min': x - l/2, 'x_max': x + l/2,
                'y_min': y - w/2, 'y_max': y + w/2,
                'z_min': z - h/2, 'z_max': z + h/2
            }

        # Get bounds for both boxes
        bounds1 = get_box_bounds(box1)
        bounds2 = get_box_bounds(box2)

        # Calculate intersection
        x_intersection = max(0, min(bounds1['x_max'], bounds2['x_max']) - 
                              max(bounds1['x_min'], bounds2['x_min']))
        y_intersection = max(0, min(bounds1['y_max'], bounds2['y_max']) - 
                              max(bounds1['y_min'], bounds2['y_min']))
        z_intersection = max(0, min(bounds1['z_max'], bounds2['z_max']) - 
                              max(bounds1['z_min'], bounds2['z_min']))
        intersection_volume = x_intersection * y_intersection * z_intersection

        # Calculate volumes of both boxes
        volume1 = box1[3] * box1[4] * box1[5]  # length * width * height
        volume2 = box2[3] * box2[4] * box2[5]  # length * width * height

        # Calculate union
        union_volume = volume1 + volume2 - intersection_volume

        # Calculate IoU
        if union_volume == 0:
            return 0.0
        
        iou = intersection_volume / union_volume

        # Add a penalty based on heading angle difference
        # This helps account for orientation differences
        heading_diff = abs(box1[6] - box2[6])
        heading_diff = min(heading_diff, 2 * np.pi - heading_diff)  # Get smallest angle
        heading_penalty = np.cos(heading_diff)  # 1 when aligned, less when misaligned
        
        # Combine IoU with heading penalty
        final_score = iou * (0.8 + 0.2 * heading_penalty)  # 80% IoU, 20% heading

        return max(0.0, min(1.0, final_score))  # Ensure result is between 0 and 1

    def evaluate_frame(self, simulator, car_detector, frame_id):
        """
        Evaluate one frame of detections
        """
          # Get data from both sources
        point_cloud, gt_boxes = self.get_simulator_data(simulator)
        detected_boxes, affinity_scores = self.get_detection_data(car_detector)

        if point_cloud is None or gt_boxes is None or detected_boxes is None or affinity_scores is None:
            print(f"Skipping frame {frame_id} due to missing data")
            return

        # Initialize frame metrics
        frame_metrics = {
            "TP": 0, "FP": 0, "FN": 0,
            "iou_scores": [], "affinity_scores": []
        }

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

        # Track matched ground truth boxes
        matched_gt = set()

        # Compare each valid detection with ground truth
        for det_idx, (det_box, score) in enumerate(zip(valid_detections, valid_scores)):
            max_iou = 0
            best_gt_idx = None

            # Find best matching ground truth box
            for gt_idx, gt_box in enumerate(gt_boxes):
                if gt_idx in matched_gt:
                    continue  # Skip already matched ground truth boxes
                
                iou = self.calculate_iou(det_box, gt_box)
                if iou > max_iou:
                    max_iou = iou
                    best_gt_idx = gt_idx

            # Store results
            self.metrics["iou_scores"].append(max_iou)
            self.metrics["affinity_scores"].append(score)
            frame_metrics["iou_scores"].append(max_iou)
            frame_metrics["affinity_scores"].append(score)

            # Update metrics
            if max_iou >= self.iou_threshold and best_gt_idx is not None:
                self.metrics["TP"] += 1
                frame_metrics["TP"] += 1
                matched_gt.add(best_gt_idx)
            else:
                self.metrics["FP"] += 1
                frame_metrics["FP"] += 1

        # Calculate false negatives (unmatched ground truth boxes)
        frame_fn = len(gt_boxes) - len(matched_gt)
        self.metrics["FN"] += frame_fn
        frame_metrics["FN"] = frame_fn

        # Store frame results
        self.frame_results[frame_id] = {
            "point_cloud": point_cloud,
            "gt_boxes": gt_boxes,
            "detected_boxes": detected_boxes,
            "affinity_scores": affinity_scores,
            "metrics": frame_metrics
        }

    def calculate_final_metrics(self):
        """
        Calculate final evaluation metrics
        """
        try:
            if (self.metrics["TP"] + self.metrics["FP"]) == 0:
                precision = 0
            else:
                precision = self.metrics["TP"] / (self.metrics["TP"] + self.metrics["FP"])
            
            if (self.metrics["TP"] + self.metrics["FN"]) == 0:
                recall = 0
            else:
                recall = self.metrics["TP"] / (self.metrics["TP"] + self.metrics["FN"])
            
            if precision + recall == 0:
                f1_score = 0
            else:
                f1_score = 2 * (precision * recall) / (precision + recall)

            return {
                "precision": precision,
                "recall": recall,
                "f1_score": f1_score,
                "average_iou": np.mean(self.metrics["iou_scores"]) if self.metrics["iou_scores"] else 0,
                "average_confidence": np.mean(self.metrics["affinity_scores"]) if self.metrics["affinity_scores"] else 0,
                "total_detections": self.metrics["total_pred"],
                "total_ground_truth": self.metrics["total_gt"],
                "true_positives": self.metrics["TP"],
                "false_positives": self.metrics["FP"],
                "false_negatives": self.metrics["FN"]
            }
        except Exception as e:
            print(f"Error calculating final metrics: {e}")
            return None

'''
    def visualize_results(self, frame_id=None):
        """
        Visualize evaluation results
        """
        try:
            # Visualize specific frame if requested
            if frame_id is not None and frame_id in self.frame_results:
                frame_data = self.frame_results[frame_id]
                draw_matplotlib(
                    frame_data["point_cloud"],
                    gt_bboxes=frame_data["gt_boxes"],
                    pred_bboxes=frame_data["detected_boxes"]
                )

            # Plot score distributions
            if self.metrics["iou_scores"] and self.metrics["affinity_scores"]:
                draw_distribution(
                    [self.metrics["iou_scores"], self.metrics["affinity_scores"]],
                    labels=["IoU Scores", "Confidence Scores"]
                )

                # Plot ROC curve
                draw_detection_roc(
                    normal_values=self.metrics["iou_scores"],
                    attack_values=self.metrics["affinity_scores"]
                )
        except Exception as e:
            print(f"Error visualizing results: {e}")
'''

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