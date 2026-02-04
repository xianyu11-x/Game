"""
Base class for avoidance algorithms
All avoidance plugins should inherit from this class
"""

class AvoidanceBase:
    """Base class for all avoidance algorithms"""
    
    def __init__(self):
        self.name = "Base Avoidance"
        self.description = "Base avoidance algorithm"
    
    def calculate_avoidance(self, circle, target_x, target_y, all_circles, delta_time):
        """
        Calculate the avoidance direction for the circle
        
        Args:
            circle: The circle that is moving (dict with x, y, radius, angle, speed, etc.)
            target_x: Target X position
            target_y: Target Y position
            all_circles: List of all circles in the scene
            delta_time: Time since last frame in seconds
        
        Returns:
            tuple: (new_x, new_y, new_angle) - New position and angle after avoidance
                   or None if no avoidance is needed
        """
        raise NotImplementedError("Subclasses must implement calculate_avoidance method")
    
    def get_name(self):
        """Return the name of the avoidance algorithm"""
        return self.name
    
    def get_description(self):
        """Return the description of the avoidance algorithm"""
        return self.description
