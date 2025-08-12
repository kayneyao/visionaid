#!/usr/bin/env python3
"""
HMM-based Traffic Light Tracker
Implements Hidden Markov Model for temporal traffic light state tracking
"""

import numpy as np
from collections import deque
import time

class HMMTrafficLightTracker:
    """Hidden Markov Model for traffic light state tracking"""
    
    def __init__(self, window_size=5, confidence_threshold=0.6):
        # State space: red, yellow, green, unknown
        self.states = ['red', 'yellow', 'green', 'unknown']
        self.n_states = len(self.states)
        self.window_size = window_size
        self.confidence_threshold = confidence_threshold
        
        # Transition matrix (self-biased to penalize rapid changes)
        self.transition_matrix = np.array([
            [0.95, 0.02, 0.02, 0.01],  # red -> [red, yellow, green, unknown]
            [0.02, 0.95, 0.02, 0.01],  # yellow -> [red, yellow, green, unknown]
            [0.02, 0.02, 0.95, 0.01],  # green -> [red, yellow, green, unknown]
            [0.01, 0.01, 0.01, 0.97]   # unknown -> [red, yellow, green, unknown]
        ])
        
        # Initial state probabilities (uniform)
        self.initial_probs = np.ones(self.n_states) / self.n_states
        
        # Current state
        self.current_state = 'unknown'
        self.current_state_idx = 3  # unknown
        
        # State history
        self.state_history = deque(maxlen=window_size)
        self.confidence_history = deque(maxlen=window_size)
        self.timestamp_history = deque(maxlen=window_size)
        
        # State change tracking
        self.last_state_change_time = time.time()
        self.state_persistence_count = 0
        self.min_persistence_frames = 3
        
        # Emission model parameters
        self.emission_std = 0.1  # Standard deviation for emission probabilities
        
    def update(self, detection_results):
        """Update tracker with new detection results"""
        current_time = time.time()
        
        # Extract traffic light detections
        traffic_light_detections = []
        for detection in detection_results:
            class_id = detection.get('class_id', -1)
            confidence = detection.get('confidence', 0.0)
            
            # Map class IDs to states
            if class_id == 7:  # redlight
                traffic_light_detections.append(('red', confidence))
            elif class_id == 10:  # yellowlight
                traffic_light_detections.append(('yellow', confidence))
            elif class_id == 4:  # greenlight
                traffic_light_detections.append(('green', confidence))
        
        if not traffic_light_detections:
            # No traffic light detected, treat as unknown
            self._update_history('unknown', 0.0, current_time)
            return self.current_state
        
        # Find the highest confidence detection
        best_detection = max(traffic_light_detections, key=lambda x: x[1])
        detected_state, confidence = best_detection
        
        # Update history
        self._update_history(detected_state, confidence, current_time)
        
        # Run HMM inference if we have enough history
        if len(self.state_history) >= self.window_size:
            self._run_hmm_inference()
        
        return self.current_state
    
    def _update_history(self, state, confidence, timestamp):
        """Update state and confidence history"""
        self.state_history.append(state)
        self.confidence_history.append(confidence)
        self.timestamp_history.append(timestamp)
    
    def _run_hmm_inference(self):
        """Run HMM inference using Viterbi algorithm"""
        # Create emission matrix from recent detections
        emission_matrix = self._create_emission_matrix()
        
        # Run Viterbi algorithm
        best_path, best_prob = self._viterbi(emission_matrix)
        
        # Get most likely state
        most_likely_state_idx = best_path[-1]
        most_likely_state = self.states[most_likely_state_idx]
        
        # Check if state change is valid
        if most_likely_state != self.current_state:
            # Validate state change
            if self._validate_state_change(most_likely_state):
                self.current_state = most_likely_state
                self.current_state_idx = most_likely_state_idx
                self.last_state_change_time = time.time()
                self.state_persistence_count = 0
            else:
                # State change rejected, maintain current state
                self.state_persistence_count += 1
        else:
            # Same state, increment persistence
            self.state_persistence_count += 1
    
    def _create_emission_matrix(self):
        """Create emission probability matrix from recent detections"""
        emission_matrix = np.zeros((self.n_states, len(self.state_history)))
        
        for t, (state, confidence) in enumerate(zip(self.state_history, self.confidence_history)):
            for s, state_name in enumerate(self.states):
                if state == state_name:
                    # High probability for detected state
                    emission_matrix[s, t] = confidence
                else:
                    # Low probability for other states
                    emission_matrix[s, t] = 0.1
        
        return emission_matrix
    
    def _viterbi(self, emission_matrix):
        """Viterbi algorithm for finding most likely state sequence"""
        T = emission_matrix.shape[1]  # Number of time steps
        N = self.n_states  # Number of states
        
        # Initialize
        delta = np.zeros((N, T))
        psi = np.zeros((N, T), dtype=int)
        
        # Initialization step
        for i in range(N):
            delta[i, 0] = np.log(self.initial_probs[i]) + np.log(emission_matrix[i, 0])
        
        # Forward pass
        for t in range(1, T):
            for j in range(N):
                # Find best previous state
                best_prev = 0
                best_prob = float('-inf')
                
                for i in range(N):
                    prob = delta[i, t-1] + np.log(self.transition_matrix[i, j])
                    if prob > best_prob:
                        best_prob = prob
                        best_prev = i
                
                delta[j, t] = best_prob + np.log(emission_matrix[j, t])
                psi[j, t] = best_prev
        
        # Backward pass
        best_path = np.zeros(T, dtype=int)
        best_path[T-1] = np.argmax(delta[:, T-1])
        
        for t in range(T-2, -1, -1):
            best_path[t] = psi[best_path[t+1], t+1]
        
        best_prob = np.max(delta[:, T-1])
        return best_path, best_prob
    
    def _validate_state_change(self, new_state):
        """Validate if state change is reasonable"""
        current_time = time.time()
        
        # Check minimum time between state changes
        if current_time - self.last_state_change_time < 2.0:  # 2 seconds minimum
            return False
        
        # Check if new state follows valid traffic light sequence
        valid_transitions = {
            'red': ['yellow', 'green'],
            'yellow': ['red'],
            'green': ['yellow', 'red'],
            'unknown': ['red', 'yellow', 'green']
        }
        
        if new_state not in valid_transitions.get(self.current_state, []):
            return False
        
        # Check confidence threshold
        recent_confidences = list(self.confidence_history)[-self.min_persistence_frames:]
        if len(recent_confidences) >= self.min_persistence_frames:
            avg_confidence = np.mean(recent_confidences)
            if avg_confidence < self.confidence_threshold:
                return False
        
        return True
    
    def get_state_confidence(self):
        """Get confidence in current state"""
        if len(self.confidence_history) == 0:
            return 0.0
        
        # Return average confidence over recent detections
        recent_confidences = list(self.confidence_history)[-5:]  # Last 5 detections
        return np.mean(recent_confidences)
    
    def get_state_persistence(self):
        """Get how long current state has been maintained"""
        if len(self.timestamp_history) == 0:
            return 0.0
        
        # Count consecutive frames with same state
        count = 0
        for state in reversed(self.state_history):
            if state == self.current_state:
                count += 1
            else:
                break
        
        return count
    
    def get_phase_change_info(self):
        """Get information about recent phase changes"""
        if len(self.timestamp_history) < 2:
            return None
        
        # Check for phase changes in recent history
        phase_changes = []
        for i in range(1, len(self.state_history)):
            if self.state_history[i] != self.state_history[i-1]:
                phase_changes.append({
                    'from_state': self.state_history[i-1],
                    'to_state': self.state_history[i],
                    'timestamp': self.timestamp_history[i]
                })
        
        return phase_changes
    
    def reset(self):
        """Reset tracker state"""
        self.current_state = 'unknown'
        self.current_state_idx = 3
        self.state_history.clear()
        self.confidence_history.clear()
        self.timestamp_history.clear()
        self.last_state_change_time = time.time()
        self.state_persistence_count = 0 