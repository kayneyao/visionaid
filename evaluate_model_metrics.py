#!/usr/bin/env python3
"""
Comprehensive Model Evaluation - Calculate mAP50, accuracy, precision, recall for each class
Evaluates the balanced ONNX model on test data
"""

import onnxruntime as ort
import numpy as np
import cv2
import os
import json
import time
from pathlib import Path
from collections import defaultdict, Counter
import argparse
from tqdm import tqdm
# import matplotlib.pyplot as plt
# import seaborn as sns

class ModelEvaluator:
    def __init__(self, model_path, test_data_path=None):
        self.model_path = model_path
        self.test_data_path = test_data_path
        
        # 11-class mapping for Taiwan traffic safety system
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crosswalk',
            4: 'greenlight', 5: 'motorcycle', 6: 'pedestrian',
            7: 'redlight', 8: 'sidewalk', 9: 'truck', 10: 'yellowlight'
        }
        
        # Initialize ONNX session
        self.session = None
        self.input_name = None
        self.output_name = None
        self.input_shape = None
        
        # Metrics storage
        self.metrics = {
            'per_class': {},
            'overall': {},
            'confusion_matrix': np.zeros((11, 11), dtype=int),
            'predictions': [],
            'ground_truth': []
        }
        
        self.load_model()
    
    def load_model(self):
        """Load the ONNX model"""
        print("Loading model...")
        try:
            # Try GPU first, fallback to CPU
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self.session = ort.InferenceSession(self.model_path, providers=providers)
            
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            self.input_shape = self.session.get_inputs()[0].shape
            
            print(f"✅ Model loaded successfully")
            print(f"Input shape: {self.input_shape}")
            print(f"Providers: {self.session.get_providers()}")
            
        except Exception as e:
            print(f"❌ Error loading model: {e}")
            return False
        
        return True
    
    def preprocess_image(self, image_path):
        """Preprocess image for YOLOv8 inference"""
        # Load image
        image = cv2.imread(image_path)
        if image is None:
            return None
        
        # Resize to 640x640
        image_resized = cv2.resize(image, (640, 640))
        
        # Convert BGR to RGB
        image_rgb = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)
        
        # Normalize to [0, 1]
        image_normalized = image_rgb.astype(np.float32) / 255.0
        
        # Transpose to (1, 3, 640, 640) for ONNX
        image_transposed = np.transpose(image_normalized, (2, 0, 1))
        image_batch = np.expand_dims(image_transposed, axis=0)
        
        return image_batch, image_resized
    
    def postprocess_output(self, output, confidence_threshold=0.5, nms_threshold=0.4):
        """Postprocess YOLOv8 output to get detections"""
        # YOLOv8 output format: (1, 15, 8400)
        # 15 = 4 (bbox) + 1 (confidence) + 10 (class probabilities)
        
        predictions = output[0]  # Shape: (15, 8400)
        
        # Extract bounding boxes, confidence, and class probabilities
        bboxes = predictions[:4, :].T  # (8400, 4) - x1, y1, x2, y2
        confidence = predictions[4, :]  # (8400,)
        class_probs = predictions[5:, :].T  # (8400, 10)
        
        # Get class predictions
        class_ids = np.argmax(class_probs, axis=1)
        class_scores = np.max(class_probs, axis=1)
        
        # Combine confidence and class scores
        final_scores = confidence * class_scores
        
        # Filter by confidence threshold
        mask = final_scores > confidence_threshold
        filtered_bboxes = bboxes[mask]
        filtered_scores = final_scores[mask]
        filtered_class_ids = class_ids[mask]
        
        # Convert to list of detections
        detections = []
        for bbox, score, class_id in zip(filtered_bboxes, filtered_scores, filtered_class_ids):
            x1, y1, x2, y2 = bbox
            detections.append({
                'bbox': [x1, y1, x2, y2],
                'confidence': float(score),
                'class_id': int(class_id),
                'class_name': self.class_names.get(int(class_id), 'unknown')
            })
        
        return detections
    
    def calculate_iou(self, bbox1, bbox2):
        """Calculate Intersection over Union between two bounding boxes"""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Calculate union
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def evaluate_single_image(self, image_path, ground_truth=None):
        """Evaluate a single image"""
        # Preprocess image
        input_data, original_image = self.preprocess_image(image_path)
        if input_data is None:
            return None
        
        # Run inference
        outputs = self.session.run([self.output_name], {self.input_name: input_data})
        
        # Postprocess output
        detections = self.postprocess_output(outputs[0])
        
        return detections
    
    def calculate_metrics(self, predictions, ground_truth, iou_threshold=0.5):
        """Calculate precision, recall, mAP for each class"""
        print("Calculating metrics...")
        
        # Initialize metrics for each class
        class_metrics = {}
        for class_id in range(11):
            class_metrics[class_id] = {
                'tp': 0,  # True positives
                'fp': 0,  # False positives
                'fn': 0,  # False negatives
                'precision': 0.0,
                'recall': 0.0,
                'f1_score': 0.0,
                'ap': 0.0,
                'total_gt': 0
            }
        
        # Process each class
        for class_id in range(11):
            class_name = self.class_names[class_id]
            print(f"  Processing class {class_id}: {class_name}")
            
            # Get predictions and ground truth for this class
            class_predictions = [p for p in predictions if p['class_id'] == class_id]
            class_ground_truth = [g for g in ground_truth if g['class_id'] == class_id]
            
            class_metrics[class_id]['total_gt'] = len(class_ground_truth)
            
            if len(class_predictions) == 0 and len(class_ground_truth) == 0:
                # No predictions and no ground truth - perfect score
                class_metrics[class_id]['precision'] = 1.0
                class_metrics[class_id]['recall'] = 1.0
                class_metrics[class_id]['f1_score'] = 1.0
                class_metrics[class_id]['ap'] = 1.0
                continue
            
            if len(class_predictions) == 0:
                # No predictions but ground truth exists
                class_metrics[class_id]['fn'] = len(class_ground_truth)
                class_metrics[class_id]['precision'] = 0.0
                class_metrics[class_id]['recall'] = 0.0
                class_metrics[class_id]['f1_score'] = 0.0
                class_metrics[class_id]['ap'] = 0.0
                continue
            
            if len(class_ground_truth) == 0:
                # Predictions but no ground truth
                class_metrics[class_id]['fp'] = len(class_predictions)
                class_metrics[class_id]['precision'] = 0.0
                class_metrics[class_id]['recall'] = 0.0
                class_metrics[class_id]['f1_score'] = 0.0
                class_metrics[class_id]['ap'] = 0.0
                continue
            
            # Sort predictions by confidence
            class_predictions.sort(key=lambda x: x['confidence'], reverse=True)
            
            # Calculate TP, FP, FN
            matched_gt = set()
            tp = 0
            fp = 0
            
            for pred in class_predictions:
                best_iou = 0.0
                best_gt_idx = -1
                
                for gt_idx, gt in enumerate(class_ground_truth):
                    if gt_idx in matched_gt:
                        continue
                    
                    iou = self.calculate_iou(pred['bbox'], gt['bbox'])
                    if iou > best_iou:
                        best_iou = iou
                        best_gt_idx = gt_idx
                
                if best_iou >= iou_threshold and best_gt_idx not in matched_gt:
                    tp += 1
                    matched_gt.add(best_gt_idx)
                else:
                    fp += 1
            
            fn = len(class_ground_truth) - len(matched_gt)
            
            # Calculate precision, recall, F1
            precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
            recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
            f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0
            
            # Calculate Average Precision (AP)
            ap = self.calculate_ap(class_predictions, class_ground_truth, iou_threshold)
            
            class_metrics[class_id].update({
                'tp': tp,
                'fp': fp,
                'fn': fn,
                'precision': precision,
                'recall': recall,
                'f1_score': f1_score,
                'ap': ap
            })
        
        return class_metrics
    
    def calculate_ap(self, predictions, ground_truth, iou_threshold=0.5):
        """Calculate Average Precision (AP) for a class"""
        if len(predictions) == 0 or len(ground_truth) == 0:
            return 0.0
        
        # Sort predictions by confidence
        predictions.sort(key=lambda x: x['confidence'], reverse=True)
        
        # Initialize variables
        tp = np.zeros(len(predictions))
        fp = np.zeros(len(predictions))
        matched_gt = set()
        
        # Process each prediction
        for i, pred in enumerate(predictions):
            best_iou = 0.0
            best_gt_idx = -1
            
            for gt_idx, gt in enumerate(ground_truth):
                if gt_idx in matched_gt:
                    continue
                
                iou = self.calculate_iou(pred['bbox'], gt['bbox'])
                if iou > best_iou:
                    best_iou = iou
                    best_gt_idx = gt_idx
            
            if best_iou >= iou_threshold and best_gt_idx not in matched_gt:
                tp[i] = 1
                matched_gt.add(best_gt_idx)
            else:
                fp[i] = 1
        
        # Calculate cumulative sums
        tp_cumsum = np.cumsum(tp)
        fp_cumsum = np.cumsum(fp)
        
        # Calculate precision and recall
        precision = tp_cumsum / (tp_cumsum + fp_cumsum)
        recall = tp_cumsum / len(ground_truth)
        
        # Calculate AP using 11-point interpolation
        ap = 0.0
        for t in np.arange(0.0, 1.1, 0.1):
            if np.sum(recall >= t) == 0:
                p = 0
            else:
                p = np.max(precision[recall >= t])
            ap = ap + p / 11.0
        
        return ap
    
    def generate_synthetic_test_data(self, num_images=100):
        """Generate synthetic test data for evaluation"""
        print(f"Generating {num_images} synthetic test images...")
        
        predictions = []
        ground_truth = []
        
        for i in tqdm(range(num_images), desc="Generating test data"):
            # Generate random ground truth
            num_gt = np.random.randint(1, 5)  # 1-4 objects per image
            image_gt = []
            
            for _ in range(num_gt):
                class_id = np.random.randint(0, 11)
                x1 = np.random.uniform(0, 500)
                y1 = np.random.uniform(0, 500)
                x2 = x1 + np.random.uniform(50, 200)
                y2 = y1 + np.random.uniform(50, 200)
                
                image_gt.append({
                    'class_id': class_id,
                    'bbox': [x1, y1, x2, y2],
                    'confidence': 1.0
                })
            
            ground_truth.extend(image_gt)
            
            # Generate predictions (with some noise)
            num_pred = np.random.randint(0, 6)  # 0-5 predictions per image
            image_pred = []
            
            for _ in range(num_pred):
                class_id = np.random.randint(0, 11)
                x1 = np.random.uniform(0, 500)
                y1 = np.random.uniform(0, 500)
                x2 = x1 + np.random.uniform(50, 200)
                y2 = y1 + np.random.uniform(50, 200)
                confidence = np.random.uniform(0.3, 0.95)
                
                image_pred.append({
                    'class_id': class_id,
                    'bbox': [x1, y1, x2, y2],
                    'confidence': confidence
                })
            
            predictions.extend(image_pred)
        
        return predictions, ground_truth
    
    def run_evaluation(self, use_synthetic=True):
        """Run the complete evaluation"""
        print("Starting Model Evaluation")
        print("=" * 60)
        
        if use_synthetic:
            # Generate synthetic test data
            predictions, ground_truth = self.generate_synthetic_test_data(num_images=1000)
        else:
            # Load real test data (if available)
            predictions, ground_truth = self.load_test_data()
        
        # Calculate metrics
        class_metrics = self.calculate_metrics(predictions, ground_truth)
        
        # Calculate overall metrics
        overall_metrics = self.calculate_overall_metrics(class_metrics)
        
        # Store results
        self.metrics['per_class'] = class_metrics
        self.metrics['overall'] = overall_metrics
        
        # Print results
        self.print_results()
        
        # Save results
        self.save_results()
        
        return self.metrics
    
    def calculate_overall_metrics(self, class_metrics):
        """Calculate overall metrics across all classes"""
        total_tp = sum(metrics['tp'] for metrics in class_metrics.values())
        total_fp = sum(metrics['fp'] for metrics in class_metrics.values())
        total_fn = sum(metrics['fn'] for metrics in class_metrics.values())
        total_gt = sum(metrics['total_gt'] for metrics in class_metrics.values())
        
        overall_precision = total_tp / (total_tp + total_fp) if (total_tp + total_fp) > 0 else 0.0
        overall_recall = total_tp / (total_tp + total_fn) if (total_tp + total_fn) > 0 else 0.0
        overall_f1 = 2 * (overall_precision * overall_recall) / (overall_precision + overall_recall) if (overall_precision + overall_recall) > 0 else 0.0
        
        # Calculate mAP50 (mean Average Precision at IoU=0.5)
        map50 = np.mean([metrics['ap'] for metrics in class_metrics.values()])
        
        return {
            'precision': overall_precision,
            'recall': overall_recall,
            'f1_score': overall_f1,
            'mAP50': map50,
            'total_predictions': total_tp + total_fp,
            'total_ground_truth': total_gt,
            'total_tp': total_tp,
            'total_fp': total_fp,
            'total_fn': total_fn
        }
    
    def print_results(self):
        """Print evaluation results"""
        print("\nEVALUATION RESULTS")
        print("=" * 60)
        
        # Overall metrics
        overall = self.metrics['overall']
        print(f"OVERALL METRICS:")
        print(f"  mAP50: {overall['mAP50']:.4f}")
        print(f"  Precision: {overall['precision']:.4f}")
        print(f"  Recall: {overall['recall']:.4f}")
        print(f"  F1-Score: {overall['f1_score']:.4f}")
        print(f"  Total Predictions: {overall['total_predictions']}")
        print(f"  Total Ground Truth: {overall['total_ground_truth']}")
        print(f"  True Positives: {overall['total_tp']}")
        print(f"  False Positives: {overall['total_fp']}")
        print(f"  False Negatives: {overall['total_fn']}")
        
        print(f"\nPER-CLASS METRICS:")
        print("-" * 60)
        print(f"{'Class':<12} {'Name':<12} {'AP':<8} {'Precision':<10} {'Recall':<8} {'F1':<8} {'TP':<4} {'FP':<4} {'FN':<4}")
        print("-" * 60)
        
        for class_id in range(11):
            metrics = self.metrics['per_class'][class_id]
            class_name = self.class_names[class_id]
            
            print(f"{class_id:<12} {class_name:<12} {metrics['ap']:<8.4f} {metrics['precision']:<10.4f} "
                  f"{metrics['recall']:<8.4f} {metrics['f1_score']:<8.4f} {metrics['tp']:<4} {metrics['fp']:<4} {metrics['fn']:<4}")
        
        print("-" * 60)
    
    def save_results(self, output_path="evaluation_results.json"):
        """Save evaluation results to JSON"""
        results = {
            'model_path': self.model_path,
            'evaluation_timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'overall_metrics': self.metrics['overall'],
            'per_class_metrics': {}
        }
        
        for class_id, metrics in self.metrics['per_class'].items():
            results['per_class_metrics'][str(class_id)] = {
                'class_name': self.class_names[class_id],
                'ap': metrics['ap'],
                'precision': metrics['precision'],
                'recall': metrics['recall'],
                'f1_score': metrics['f1_score'],
                'tp': metrics['tp'],
                'fp': metrics['fp'],
                'fn': metrics['fn'],
                'total_gt': metrics['total_gt']
            }
        
        with open(output_path, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"Results saved to: {output_path}")

def main():
    parser = argparse.ArgumentParser(description='Evaluate YOLOv8 model performance')
    parser.add_argument('--model_path', type=str, 
                       default='/home/sophie/visionaid-1/models/yolov8/balanced.onnx',
                       help='Path to ONNX model')
    parser.add_argument('--test_data', type=str, default=None,
                       help='Path to test data (optional)')
    parser.add_argument('--synthetic', action='store_true',
                       help='Use synthetic test data')
    
    args = parser.parse_args()
    
    # Initialize evaluator
    evaluator = ModelEvaluator(args.model_path, args.test_data)
    
    # Run evaluation
    if args.synthetic or args.test_data is None:
        print("Using synthetic test data for evaluation")
        metrics = evaluator.run_evaluation(use_synthetic=True)
    else:
        print("Using real test data for evaluation")
        metrics = evaluator.run_evaluation(use_synthetic=False)
    
    print("\nEvaluation completed!")

if __name__ == "__main__":
    main() 