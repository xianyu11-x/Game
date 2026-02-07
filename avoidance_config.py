"""
Centralized configuration for avoidance algorithms.
"""

import math

# Global debug toggle for all avoidance algorithms
DEBUG_ENABLED = False

DEFAULT_AVOIDANCE_ALGORITHM = "Predictive Scan"

AVOIDANCE_CONFIG = {
    "predictive_scan": {
        "name": "Predictive Scan",
        "description": "Predicts collisions by scanning candidate angles within a look-ahead circle",
        # Core prediction settings
        "prediction_time": 0.5,
        "speed_override": None,  # Set to a number to override circle["speed"]
        "speed_factor": 1.0,     # Multiplies circle["speed"] when no override is set
        # Detection radius derived from speed * prediction_time
        "detection_radius_factor": 1.5,
        "detection_radius_min": 40.0,
        "detection_radius_max": 260.0,
    # Angle scanning (debug-only: used for visualization sampling)
    "angle_step_deg": 10.0,
        # Collision checking
        "collision_padding": 2.0,
        "neighbor_speed_factor": 1.0,
    # Parametric time sampling for feasibility solving
    "time_samples": 16,
    # Turn penalty to discourage large heading changes
    "turn_penalty_weight": 0.6,
    "turn_penalty_threshold_deg": 45.0,
    # Opposite direction suppression
    "opposite_suppression_enabled": True,
    "opposite_angle_threshold_deg": 30.0,
    "opposite_preference_margin": 15.0,
    # Reverse cooldown (oscillation suppression)
    "reverse_cooldown_enabled": True,
    "reverse_cooldown_base": 0.4,
    "reverse_cooldown_max": 5,
    "reverse_cooldown_growth": 2.0,
    "reverse_cooldown_decay": 0.8,
    "reverse_extreme_threshold_deg": 120.0,
    # Concave detection (increase prediction time when stuck)
    "concave_detection_enabled": True,
    "concave_progress_epsilon": 1.5,
    "concave_time_growth": 0.6,
    "concave_time_max": 4.0,
    "concave_decay": 0.8,
        # Angle selection strategy
        "angle_selection_method": "closest_to_target",
        # Fallback when no angles are available: keep_current | stop | target
        "fallback_method": "keep_current",
    }
    ,
    "predictive_scan_parallel": {
        "name": "Predictive Scan (Parallel)",
        "description": "Predictive scan with parallel blocked interval evaluation",
        # Core prediction settings
        "prediction_time": 0.5,
        "speed_override": None,
        "speed_factor": 1.0,
        # Detection radius derived from speed * prediction_time
        "detection_radius_factor": 1.5,
        "detection_radius_min": 40.0,
        "detection_radius_max": 260.0,
        # Angle scanning (debug-only: used for visualization sampling)
        "angle_step_deg": 10.0,
        # Collision checking
        "collision_padding": 2.0,
        "neighbor_speed_factor": 1.0,
        # Parametric time sampling for feasibility solving
        "time_samples": 16,
        # Parallelization
        "parallel_workers": 0,
        # Turn penalty to discourage large heading changes
        "turn_penalty_weight": 0.6,
        "turn_penalty_threshold_deg": 45.0,
        # Opposite direction suppression
        "opposite_suppression_enabled": True,
        "opposite_angle_threshold_deg": 30.0,
        "opposite_preference_margin": 15.0,
        # Reverse cooldown (oscillation suppression)
        "reverse_cooldown_enabled": True,
        "reverse_cooldown_base": 0.4,
        "reverse_cooldown_max": 5,
        "reverse_cooldown_growth": 2.0,
        "reverse_cooldown_decay": 0.8,
        "reverse_extreme_threshold_deg": 120.0,
        # Concave detection (increase prediction time when stuck)
        "concave_detection_enabled": True,
        "concave_progress_epsilon": 1.5,
        "concave_time_growth": 0.6,
        "concave_time_max": 4.0,
        "concave_decay": 0.8,
        # Angle selection strategy
        "angle_selection_method": "closest_to_target",
        # Fallback when no angles are available: keep_current | stop | target
        "fallback_method": "keep_current",
    }
    ,
    "predictive_scan_vectorized": {
        "name": "Predictive Scan (Vectorized)",
        "description": "Predictive scan with NumPy batch evaluation",
        # Core prediction settings
        "prediction_time": 0.5,
        "speed_override": None,
        "speed_factor": 1.0,
        # Detection radius derived from speed * prediction_time
        "detection_radius_factor": 1.5,
        "detection_radius_min": 40.0,
        "detection_radius_max": 260.0,
        # Angle scanning (debug-only: used for visualization sampling)
        "angle_step_deg": 10.0,
        # Collision checking
        "collision_padding": 2.0,
        "neighbor_speed_factor": 1.0,
        # Parametric time sampling for feasibility solving
        "time_samples": 16,
        # Turn penalty to discourage large heading changes
        "turn_penalty_weight": 0.6,
        "turn_penalty_threshold_deg": 45.0,
        # Opposite direction suppression
        "opposite_suppression_enabled": True,
        "opposite_angle_threshold_deg": 30.0,
        "opposite_preference_margin": 15.0,
        # Reverse cooldown (oscillation suppression)
        "reverse_cooldown_enabled": True,
        "reverse_cooldown_base": 0.4,
        "reverse_cooldown_max": 5,
        "reverse_cooldown_growth": 2.0,
        "reverse_cooldown_decay": 0.8,
        "reverse_extreme_threshold_deg": 120.0,
        # Concave detection (increase prediction time when stuck)
        "concave_detection_enabled": True,
        "concave_progress_epsilon": 1.5,
        "concave_time_growth": 0.6,
        "concave_time_max": 4.0,
        "concave_decay": 0.8,
        # Angle selection strategy
        "angle_selection_method": "closest_to_target",
        # Fallback when no angles are available: keep_current | stop | target
        "fallback_method": "keep_current",
    }
}


def normalize_angle(angle):
    """Normalize angle to [-180, 180)."""
    normalized = angle % 360
    if normalized >= 180:
        normalized -= 360
    return normalized


def angular_distance(angle_a, angle_b):
    """Return absolute shortest distance between two angles in degrees."""
    diff = normalize_angle(angle_a - angle_b)
    return abs(diff)


def select_angle_closest_to_target(angles, target_angle, current_angle):
    """Select the angle closest to the target direction."""
    if not angles:
        return None
    return min(angles, key=lambda a: angular_distance(a, target_angle))


def select_angle_closest_to_current(angles, target_angle, current_angle):
    """Select the angle closest to the current direction."""
    if not angles:
        return None
    return min(angles, key=lambda a: angular_distance(a, current_angle))


def select_angle_weighted_target_current(angles, target_angle, current_angle, target_weight=0.7):
    """Select the angle by a weighted score of target and current direction."""
    if not angles:
        return None

    def score(angle):
        return (angular_distance(angle, target_angle) * target_weight +
                angular_distance(angle, current_angle) * (1.0 - target_weight))

    return min(angles, key=score)


ANGLE_SELECTION_FUNCTIONS = {
    "closest_to_target": select_angle_closest_to_target,
    "closest_to_current": select_angle_closest_to_current,
    "weighted_target_current": select_angle_weighted_target_current,
}
