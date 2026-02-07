import math
import unittest

from avoidance_predictive import PredictiveScanAvoidance


class TestPredictiveScanAvoidance(unittest.TestCase):
    def setUp(self):
        self.avoidance = PredictiveScanAvoidance()
        # Configure tighter, deterministic settings for tests
        self.avoidance.config.update({
            "prediction_time": 1.0,
            "speed_override": None,
            "speed_factor": 1.0,
            "detection_radius_factor": 2.0,
            "detection_radius_min": 0.0,
            "detection_radius_max": 9999.0,
            "angle_step_deg": 45.0,
            "collision_padding": 0.0,
            "angle_selection_method": "closest_to_target",
            "fallback_method": "keep_current",
            "neighbor_speed_factor": 0.0,
            "concave_detection_enabled": False,
        })

    def test_blocks_colliding_angle(self):
        circle = {"x": 0.0, "y": 0.0, "radius": 5.0, "angle": 0.0, "speed": 10.0}
        target_x, target_y = 100.0, 0.0
        blocker = {"x": 15.0, "y": 0.0, "radius": 5.0, "angle": 0.0, "speed": 0.0}

        new_x, new_y, new_angle = self.avoidance.calculate_avoidance(
            circle, target_x, target_y, [circle, blocker], 0.1
        )

        self.assertNotEqual(new_angle, 0.0)
        self.assertTrue(math.hypot(new_x - circle["x"], new_y - circle["y"]) > 0)

    def test_stationary_neighbor_speed_is_zero(self):
        circle = {"x": 0.0, "y": 0.0, "radius": 5.0, "angle": 0.0, "speed": 10.0}
        neighbor = {"x": 20.0, "y": 0.0, "radius": 5.0, "angle": 0.0, "speed": 50.0}

        speed = self.avoidance._get_neighbor_speed(neighbor)
        self.assertEqual(speed, 0.0)

    def test_chooses_target_angle_when_clear(self):
        circle = {"x": 0.0, "y": 0.0, "radius": 5.0, "angle": 30.0, "speed": 10.0}
        target_x, target_y = 100.0, 0.0

        new_x, new_y, new_angle = self.avoidance.calculate_avoidance(
            circle, target_x, target_y, [circle], 0.1
        )

        self.assertAlmostEqual(new_angle, 0.0, delta=1e-6)
        self.assertTrue(math.hypot(new_x - circle["x"], new_y - circle["y"]) > 0)


if __name__ == "__main__":
    unittest.main()
