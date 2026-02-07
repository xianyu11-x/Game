"""
Predictive scan avoidance algorithm with NumPy batch evaluation.
"""

from __future__ import annotations

import math

from avoidance_config import AVOIDANCE_CONFIG, normalize_angle
from avoidance_predictive import PredictiveScanAvoidance

try:
    import numpy as np
except ImportError:  # pragma: no cover - optional dependency
    np = None


class PredictiveScanAvoidanceVectorized(PredictiveScanAvoidance):
    """Predictive scan avoidance with vectorized blocked interval evaluation."""

    def __init__(self):
        super().__init__()
        self.config = AVOIDANCE_CONFIG["predictive_scan_vectorized"]
        self.name = self.config["name"]
        self.description = self.config["description"]

    def _get_blocked_intervals(self, circle, neighbors, self_speed, prediction_time):
        if not neighbors:
            return []
        if np is None:
            return super()._get_blocked_intervals(circle, neighbors, self_speed, prediction_time)

        time_samples = max(4, int(self.config.get("time_samples", 16)))
        t_values = prediction_time * np.arange(1, time_samples + 1, dtype=float) / time_samples
        self_distances = self_speed * t_values

        blocked_intervals = []
        for other in neighbors:
            other_speed = self._get_neighbor_speed(other)
            other_angle = math.radians(other["angle"])
            v_other = (math.cos(other_angle) * other_speed, math.sin(other_angle) * other_speed)

            p0 = (other["x"] - circle["x"], other["y"] - circle["y"])
            base_distance = math.hypot(p0[0], p0[1])
            combined_radius = circle["radius"] + other["radius"] + self.config["collision_padding"]

            if base_distance <= 1e-6:
                return [(-180.0, 180.0)]

            qx = p0[0] + v_other[0] * t_values
            qy = p0[1] + v_other[1] * t_values
            distance = np.hypot(qx, qy)

            valid_mask = self_distances > 1e-6
            if not np.any(valid_mask):
                continue

            abs_diff = np.abs(self_distances - combined_radius)
            contains_mask = (distance < abs_diff) & (combined_radius >= self_distances + distance)
            if np.any(contains_mask & valid_mask):
                return [(-180.0, 180.0)]

            intersect_mask = (
                (distance <= self_distances + combined_radius)
                & (distance >= abs_diff)
                & valid_mask
            )
            if not np.any(intersect_mask):
                continue

            d = distance[intersect_mask]
            s = self_distances[intersect_mask]
            qx_sel = qx[intersect_mask]
            qy_sel = qy[intersect_mask]

            cos_half = (d**2 + s**2 - combined_radius**2) / (2 * d * s)
            cos_half = np.clip(cos_half, -1.0, 1.0)
            half_angle = np.degrees(np.arccos(cos_half))

            center_angle = _normalize_angle_array(np.degrees(np.arctan2(qy_sel, qx_sel)))
            start = _normalize_angle_array(center_angle - half_angle)
            end = _normalize_angle_array(center_angle + half_angle)

            for start_angle, end_angle in zip(start, end):
                if start_angle <= end_angle:
                    blocked_intervals.append((float(start_angle), float(end_angle)))
                else:
                    blocked_intervals.append((float(start_angle), 180.0))
                    blocked_intervals.append((-180.0, float(end_angle)))

        return self._merge_intervals(blocked_intervals)


def _normalize_angle_array(angles):
    if np is None:
        return angles
    return ((angles + 180.0) % 360.0) - 180.0
