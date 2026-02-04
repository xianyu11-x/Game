"""
Repulsion-based avoidance algorithm
Steers away from nearby obstacles
"""

from avoidance_base import AvoidanceBase
import math

class RepulsionAvoidance(AvoidanceBase):
    """Repulsion avoidance - steers away from nearby obstacles"""
    
    def __init__(self):
        super().__init__()
        self.name = "Repulsion"
        self.description = "Steers away from nearby obstacles"
        self.detection_radius = 60  # How far to detect obstacles
        self.repulsion_strength = 3.0  # How strong the repulsion force is
    
    def calculate_avoidance(self, circle, target_x, target_y, all_circles, delta_time):
        """Calculate avoidance using repulsion forces"""
        # Calculate base movement direction
        dx = target_x - circle["x"]
        dy = target_y - circle["y"]
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        if distance_to_target < 0.1:
            return (circle["x"], circle["y"], circle["angle"])
        
        # Normalize direction to target
        target_dir_x = dx / distance_to_target
        target_dir_y = dy / distance_to_target
        
        # Calculate repulsion forces from nearby obstacles
        repulsion_x = 0
        repulsion_y = 0
        
        for other in all_circles:
            if other is circle:
                continue
            
            # Calculate distance to obstacle
            to_obstacle_x = other["x"] - circle["x"]
            to_obstacle_y = other["y"] - circle["y"]
            distance = math.sqrt(to_obstacle_x**2 + to_obstacle_y**2)
            
            # Only consider obstacles within detection radius
            if distance < self.detection_radius and distance > 0:
                # Calculate repulsion force (stronger when closer)
                force = (1.0 - distance / self.detection_radius) * self.repulsion_strength
                
                # Add repulsion away from obstacle
                repulsion_x -= (to_obstacle_x / distance) * force
                repulsion_y -= (to_obstacle_y / distance) * force
        
        # Combine target direction with repulsion
        final_dir_x = target_dir_x + repulsion_x
        final_dir_y = target_dir_y + repulsion_y
        
        # Normalize final direction
        final_distance = math.sqrt(final_dir_x**2 + final_dir_y**2)
        if final_distance > 0:
            final_dir_x /= final_distance
            final_dir_y /= final_distance
        
        # Calculate new angle
        new_angle = math.degrees(math.atan2(final_dir_y, final_dir_x))
        
        # Calculate new position
        move_distance = circle["speed"] * delta_time
        new_x = circle["x"] + move_distance * final_dir_x
        new_y = circle["y"] + move_distance * final_dir_y
        
        # Check if new position collides
        collision = False
        for other in all_circles:
            if other is circle:
                continue
            dist = math.sqrt((new_x - other["x"])**2 + (new_y - other["y"])**2)
            if dist < (circle["radius"] + other["radius"]):
                collision = True
                break
        
        # If collision detected, don't move
        if collision:
            return (circle["x"], circle["y"], new_angle)
        
        return (new_x, new_y, new_angle)
