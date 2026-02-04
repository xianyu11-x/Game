"""
Simple sliding avoidance algorithm
Tries to slide along obstacles when collision is detected
"""

from avoidance_base import AvoidanceBase
import math

class SimpleAvoidance(AvoidanceBase):
    """Simple sliding avoidance - tries to slide along obstacles"""
    
    def __init__(self):
        super().__init__()
        self.name = "Simple Sliding"
        self.description = "Slides along obstacles when collision detected"
    
    def check_collision(self, x1, y1, radius1, x2, y2, radius2):
        """Check if two circles collide"""
        distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        return distance < (radius1 + radius2)
    
    def calculate_avoidance(self, circle, target_x, target_y, all_circles, delta_time):
        """Calculate avoidance using simple sliding"""
        # Calculate movement
        move_distance = circle["speed"] * delta_time
        new_x = circle["x"] + move_distance * math.cos(math.radians(circle["angle"]))
        new_y = circle["y"] + move_distance * math.sin(math.radians(circle["angle"]))
        
        # Check for collisions
        collision_detected = False
        for other in all_circles:
            if other is circle:
                continue
            
            if self.check_collision(new_x, new_y, circle["radius"], 
                                   other["x"], other["y"], other["radius"]):
                collision_detected = True
                break
        
        # If no collision, move normally
        if not collision_detected:
            return (new_x, new_y, circle["angle"])
        
        # Try sliding along X axis
        test_x = circle["x"] + move_distance * math.cos(math.radians(circle["angle"]))
        can_move_x = True
        for other in all_circles:
            if other is circle:
                continue
            if self.check_collision(test_x, circle["y"], circle["radius"],
                                   other["x"], other["y"], other["radius"]):
                can_move_x = False
                break
        
        # Try sliding along Y axis
        test_y = circle["y"] + move_distance * math.sin(math.radians(circle["angle"]))
        can_move_y = True
        for other in all_circles:
            if other is circle:
                continue
            if self.check_collision(circle["x"], test_y, circle["radius"],
                                   other["x"], other["y"], other["radius"]):
                can_move_y = False
                break
        
        # Apply sliding
        result_x = test_x if can_move_x else circle["x"]
        result_y = test_y if can_move_y else circle["y"]
        
        return (result_x, result_y, circle["angle"])
