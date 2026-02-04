import pygame
import sys
import math
from avoidance_manager import AvoidanceManager

# Initialize Pygame
pygame.init()

# Window setup
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("2D Game - WASD Camera Movement")

# Color definitions
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (100, 149, 237)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)

# Camera settings
camera_x = 0
camera_y = 0
camera_speed = 5

# Store placed circles
circles = []  # Each element: {"x": x, "y": y, "label": label, "target_x": None, "target_y": None, "angle": 0, "speed": 100, "turn_speed": 180, "radius": 25}
circle_counter = 0
selected_circles = []  # Changed to list
global_target = None   # (x, y)
is_dragging = False
drag_start_pos = (0, 0)

# Initialize avoidance manager
avoidance_manager = AvoidanceManager()
plugin_count = avoidance_manager.load_plugins()
print(f"Loaded {plugin_count} avoidance plugins")
print(f"Available algorithms: {avoidance_manager.get_algorithm_names()}")
print(f"Current algorithm: {avoidance_manager.get_algorithm().get_name() if avoidance_manager.get_algorithm() else 'None'}")

# Font setup
font = pygame.font.Font(None, 24)
small_font = pygame.font.Font(None, 18)

# Clock for frame rate control
clock = pygame.time.Clock()

def check_collision(x1, y1, radius1, x2, y2, radius2):
    """Check if two circles collide"""
    distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    return distance < (radius1 + radius2)

def get_collision_response(circle, other_circles):
    """Calculate collision response to push circles apart"""
    import math
    push_x = 0
    push_y = 0
    
    for other in other_circles:
        if other is circle:
            continue
        
        distance = ((other["x"] - circle["x"]) ** 2 + (other["y"] - circle["y"]) ** 2) ** 0.5
        overlap = (circle["radius"] + other["radius"]) - distance
        
        if overlap > 0:
            # Calculate push direction (away from other circle)
            if distance > 0:
                push_x -= (other["x"] - circle["x"]) / distance * overlap * 0.5
                push_y -= (other["y"] - circle["y"]) / distance * overlap * 0.5
    
    return push_x, push_y

def move_circle_towards_target(circle, delta_time, all_circles, avoidance_algorithm):
    """Move circle towards its target position with speed and rotation, using avoidance plugin"""
    if circle["target_x"] is None or circle["target_y"] is None:
        return
    
    # Calculate direction to target
    dx = circle["target_x"] - circle["x"]
    dy = circle["target_y"] - circle["y"]
    distance = (dx ** 2 + dy ** 2) ** 0.5
    
    # Check if reached target
    # Default to precise arrival (distance < 5) if not specified
    arrival_threshold = circle.get("arrival_threshold", 5)
    
    if distance < arrival_threshold:
        circle["target_x"] = None
        circle["target_y"] = None
        return
    
    # Calculate target angle (in degrees)
    target_angle = math.degrees(math.atan2(dy, dx))
    
    # Normalize angles to -180 to 180
    current_angle = circle["angle"] % 360
    if current_angle > 180:
        current_angle -= 360
    
    target_angle = target_angle % 360
    if target_angle > 180:
        target_angle -= 360
    
    # Calculate angle difference
    angle_diff = target_angle - current_angle
    if angle_diff > 180:
        angle_diff -= 360
    elif angle_diff < -180:
        angle_diff += 360
    
    # Rotate towards target
    max_rotation = circle["turn_speed"] * delta_time
    if abs(angle_diff) < max_rotation:
        circle["angle"] = target_angle
    else:
        circle["angle"] += max_rotation if angle_diff > 0 else -max_rotation
    
    # Use avoidance algorithm if available
    if avoidance_algorithm:
        result = avoidance_algorithm.calculate_avoidance(
            circle, circle["target_x"], circle["target_y"], all_circles, delta_time
        )
        if result:
            new_x, new_y, new_angle = result
            circle["x"] = new_x
            circle["y"] = new_y
            # Optionally update angle from avoidance algorithm
            # circle["angle"] = new_angle
    else:
        # Fallback to simple movement
        move_distance = circle["speed"] * delta_time
        circle["x"] += move_distance * math.cos(math.radians(circle["angle"]))
        circle["y"] += move_distance * math.sin(math.radians(circle["angle"]))
    
    # Apply collision response to separate overlapping circles
    # 只对有目标的圆形应用碰撞响应，避免目标点被推动
    if circle["target_x"] is not None and circle["target_y"] is not None:
        push_x, push_y = get_collision_response(circle, all_circles)
        circle["x"] += push_x
        circle["y"] += push_y

def draw_grid(surface, camera_x, camera_y):
    """Draw grid background"""
    grid_size = 50
    # Vertical lines
    for x in range(-camera_x % grid_size, WINDOW_WIDTH, grid_size):
        pygame.draw.line(surface, (200, 200, 200), (x, 0), (x, WINDOW_HEIGHT), 1)
    # Horizontal lines
    for y in range(-camera_y % grid_size, WINDOW_HEIGHT, grid_size):
        pygame.draw.line(surface, (200, 200, 200), (0, y), (WINDOW_WIDTH, y), 1)

def world_to_screen(world_x, world_y, camera_x, camera_y):
    """Convert world coordinates to screen coordinates"""
    screen_x = world_x - camera_x
    screen_y = world_y - camera_y
    return screen_x, screen_y

def screen_to_world(screen_x, screen_y, camera_x, camera_y):
    """Convert screen coordinates to world coordinates"""
    world_x = screen_x + camera_x
    world_y = screen_y + camera_y
    return world_x, world_y

def get_circle_at_position(world_x, world_y, circles, radius=25):
    """Check if there's a circle at the given world position"""
    for circle in circles:
        distance = ((circle["x"] - world_x) ** 2 + (circle["y"] - world_y) ** 2) ** 0.5
        if distance <= radius:
            return circle
    return None

# Main game loop
running = True
while running:
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        # Mouse handling
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left click
                is_dragging = True
                drag_start_pos = pygame.mouse.get_pos()
            
            elif event.button == 3:  # Right click
                # Move selected circles to mouse position (Precise Arrival)
                if selected_circles:
                    
                    # Clear global target marking (cancel Z-target)
                    global_target = None
                    
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    world_x, world_y = screen_to_world(mouse_x, mouse_y, camera_x, camera_y)
                    
                    # Store this just for visualization if needed, or just set targets
                    # Note: We don't have a single "right_click_target" variable to draw, 
                    # but we can set individual targets
                    
                    for circle in selected_circles:
                        circle["target_x"] = world_x
                        circle["target_y"] = world_y
                        circle["arrival_threshold"] = 5  # Precise arrival
                        
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:  # Left click release
                is_dragging = False
                drag_end_pos = pygame.mouse.get_pos()
                
                # Check distance to distinguish click vs drag
                dist = math.hypot(drag_end_pos[0] - drag_start_pos[0], drag_end_pos[1] - drag_start_pos[1])
                
                if dist < 5:  # Click
                    mouse_x, mouse_y = drag_end_pos
                    world_x, world_y = screen_to_world(mouse_x, mouse_y, camera_x, camera_y)
                    clicked_circle = get_circle_at_position(world_x, world_y, circles)
                    
                    if clicked_circle:
                        selected_circles = [clicked_circle]  # Single select
                    else:
                         selected_circles = [] # Deselect if clicking empty space

                else:
                    # Drag selection - Box select
                    start_world = screen_to_world(drag_start_pos[0], drag_start_pos[1], camera_x, camera_y)
                    end_world = screen_to_world(drag_end_pos[0], drag_end_pos[1], camera_x, camera_y)
                    
                    left = min(start_world[0], end_world[0])
                    right = max(start_world[0], end_world[0])
                    top = min(start_world[1], end_world[1])
                    bottom = max(start_world[1], end_world[1])
                    
                    selected_circles = []
                    for circle in circles:
                        if left < circle["x"] < right and top < circle["y"] < bottom:
                            selected_circles.append(circle)
        
        # Keyboard shortcuts
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                # Place new circle at mouse position
                mouse_x, mouse_y = pygame.mouse.get_pos()
                world_x, world_y = screen_to_world(mouse_x, mouse_y, camera_x, camera_y)
                
                circle_counter += 1
                new_circle = {
                    "x": world_x,
                    "y": world_y,
                    "label": f"Circle{circle_counter}",
                    "target_x": None,
                    "target_y": None,
                    "angle": 0,
                    "speed": 100,
                    "turn_speed": 180,
                    "radius": 25,
                    "arrival_threshold": 5
                }
                
                can_place = True
                for other in circles:
                    if check_collision(new_circle["x"], new_circle["y"], new_circle["radius"],
                                     other["x"], other["y"], other["radius"]):
                        can_place = False
                        break
                
                if can_place:
                    circles.append(new_circle)

            if event.key == pygame.K_z:
                # Set target to an existing circle
                mouse_x, mouse_y = pygame.mouse.get_pos()
                world_x, world_y = screen_to_world(mouse_x, mouse_y, camera_x, camera_y)
                
                target_unit = get_circle_at_position(world_x, world_y, circles)
                
                if target_unit:
                    global_target = (target_unit["x"], target_unit["y"])
                    print(f"Target set to {target_unit['label']}")
                    
                    if selected_circles:
                        for circle in selected_circles:
                            if circle is target_unit:
                                continue
                            circle["target_x"] = target_unit["x"]
                            circle["target_y"] = target_unit["y"]
                            # Arrive when touching (Radius + Radius + Margin)
                            # 这样可以实现多个单位围住一个目标
                            circle["arrival_threshold"] = circle["radius"] + target_unit["radius"] + 5
                
            if event.key == pygame.K_TAB:
                # Switch to next avoidance algorithm
                avoidance_manager.next_algorithm()
                current_algo = avoidance_manager.get_algorithm()
                print(f"Switched to: {current_algo.get_name() if current_algo else 'None'}")
    
    # Keyboard input - WASD camera movement
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        camera_y -= camera_speed
    if keys[pygame.K_s]:
        camera_y += camera_speed
    if keys[pygame.K_a]:
        camera_x -= camera_speed
    if keys[pygame.K_d]:
        camera_x += camera_speed
    
    # Update circle movements
    delta_time = clock.get_time() / 1000.0  # Convert to seconds
    current_algorithm = avoidance_manager.get_algorithm()
    for circle in circles:
        move_circle_towards_target(circle, delta_time, circles, current_algorithm)
    
    # Clear screen
    screen.fill(WHITE)
    
    # Draw grid
    draw_grid(screen, camera_x, camera_y)
    
    # Draw all circles
    for circle in circles:
        # Convert to screen coordinates
        screen_x, screen_y = world_to_screen(circle["x"], circle["y"], camera_x, camera_y)
        
        # Only draw circles within screen bounds (optimization)
        if -50 < screen_x < WINDOW_WIDTH + 50 and -50 < screen_y < WINDOW_HEIGHT + 50:
            # Determine color based on selection
            circle_color = GREEN if circle in selected_circles else BLUE
            
            # Draw circle
            pygame.draw.circle(screen, circle_color, (int(screen_x), int(screen_y)), circle["radius"])
            pygame.draw.circle(screen, BLACK, (int(screen_x), int(screen_y)), circle["radius"], 2)
            
            # Draw direction indicator (arrow)
            import math
            arrow_length = 20
            arrow_x = screen_x + arrow_length * math.cos(math.radians(circle["angle"]))
            arrow_y = screen_y + arrow_length * math.sin(math.radians(circle["angle"]))
            pygame.draw.line(screen, RED, (int(screen_x), int(screen_y)), 
                           (int(arrow_x), int(arrow_y)), 3)
            
            # Draw label
            label_surface = font.render(circle["label"], True, BLACK)
            label_rect = label_surface.get_rect(center=(int(screen_x), int(screen_y - 35)))
            screen.blit(label_surface, label_rect)
            
            # Draw target position if exists (individual target)
            if circle["target_x"] is not None and circle["target_y"] is not None:
                target_screen_x, target_screen_y = world_to_screen(
                    circle["target_x"], circle["target_y"], camera_x, camera_y
                )
                # Draw small x marker at target
                pygame.draw.line(screen, (200, 100, 100), 
                               (int(target_screen_x - 5), int(target_screen_y - 5)),
                               (int(target_screen_x + 5), int(target_screen_y + 5)), 1)
                pygame.draw.line(screen, (200, 100, 100),
                               (int(target_screen_x + 5), int(target_screen_y - 5)),
                               (int(target_screen_x - 5), int(target_screen_y + 5)), 1)

    # Draw global target (Z key)
    if global_target:
        gt_screen_x, gt_screen_y = world_to_screen(global_target[0], global_target[1], camera_x, camera_y)
        # Draw a distinctive marker (Magenta X with Circle)
        pygame.draw.circle(screen, (255, 0, 255), (int(gt_screen_x), int(gt_screen_y)), 15, 2)
        pygame.draw.line(screen, (255, 0, 255), (int(gt_screen_x)-20, int(gt_screen_y)-20), (int(gt_screen_x)+20, int(gt_screen_y)+20), 3)
        pygame.draw.line(screen, (255, 0, 255), (int(gt_screen_x)+20, int(gt_screen_y)-20), (int(gt_screen_x)-20, int(gt_screen_y)+20), 3)
        
        # Label for target
        target_label = font.render("Target (Z)", True, (255, 0, 255))
        screen.blit(target_label, (int(gt_screen_x) + 20, int(gt_screen_y) - 20))

    # Draw drag selection box
    if is_dragging:
        mouse_x, mouse_y = pygame.mouse.get_pos()
        rect_left = min(drag_start_pos[0], mouse_x)
        rect_top = min(drag_start_pos[1], mouse_y)
        rect_width = abs(drag_start_pos[0] - mouse_x)
        rect_height = abs(drag_start_pos[1] - mouse_y)
        
        selection_rect = pygame.Rect(rect_left, rect_top, rect_width, rect_height)
        pygame.draw.rect(screen, GREEN, selection_rect, 1) # Thin green border
        
        # Transparent fill
        s = pygame.Surface((rect_width, rect_height))
        s.set_alpha(50)
        s.fill(GREEN)
        screen.blit(s, (rect_left, rect_top))

    # Draw info text
    info_text = [
        "Controls:",
        " [Q] Place Unit",
        " [Left Drag] Select Units",
        " [Right Click] Move Here (Precise)",
        " [Z] Attack/Follow Unit (Touch)",
        " [TAB] Switch Algorithm",
        " [WASD] Camera",
        "",
        f"Camera: ({camera_x}, {camera_y})",
        f"Circles: {len(circles)}",
        f"Selected: {len(selected_circles)}",
        f"Algorithm: {avoidance_manager.get_algorithm().get_name() if avoidance_manager.get_algorithm() else 'None'}"
    ]
    
    y_offset = 10
    for text in info_text:
        text_surface = font.render(text, True, BLACK)
        text_rect = pygame.Rect(10, y_offset, 300, 30)
        pygame.draw.rect(screen, (255, 255, 255, 200), text_rect)
        screen.blit(text_surface, (15, y_offset))
        y_offset += 25
    
    # Update display
    pygame.display.flip()
    
    # Control frame rate
    clock.tick(60)

# Exit
pygame.quit()
sys.exit()
