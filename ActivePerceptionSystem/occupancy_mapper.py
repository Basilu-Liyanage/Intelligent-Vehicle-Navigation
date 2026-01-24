import numpy as np
import math

class OccupancyMapper:
    """
    Creates 2D occupancy grid from LiDAR scans
    """
    
    def __init__(self, grid_size=120, resolution=5):
        """
        grid_size: cm (120cm = 1.2m radius)
        resolution: cm per grid cell
        """
        self.grid_size = grid_size
        self.resolution = resolution
        self.grid_cells = int(grid_size / resolution)
        
        # Create 2D grid (0=free, 1=occupied, 0.5=unknown)
        self.grid = np.ones((self.grid_cells, self.grid_cells)) * 0.5
        
        # Vehicle position at center of grid
        self.vehicle_x = self.grid_cells // 2
        self.vehicle_y = self.grid_cells // 2
        
    def update_from_scan(self, scan_data, vehicle_angle=0):
        """
        Update occupancy grid with new scan data
        scan_data: [(angle1, dist1), (angle2, dist2), ...]
        vehicle_angle: Current heading of vehicle (0=straight)
        """
        # Clear old observations (optional: add decay instead)
        self.grid = self.grid * 0.7  # Decay old observations
        
        for scan_angle, distance_cm in scan_data:
            if distance_cm > self.grid_size:
                distance_cm = self.grid_size  # Cap at grid size
            
            # Convert to global angle (add vehicle orientation)
            global_angle = scan_angle + vehicle_angle - 25  # Adjust for center
            
            # Convert polar to cartesian (relative to vehicle)
            rad = math.radians(global_angle)
            x_rel = distance_cm * math.cos(rad)
            y_rel = distance_cm * math.sin(rad)
            
            # Convert to grid coordinates
            x_grid = self.vehicle_x + int(x_rel / self.resolution)
            y_grid = self.vehicle_y + int(y_rel / self.resolution)
            
            # Ensure within bounds
            if 0 <= x_grid < self.grid_cells and 0 <= y_grid < self.grid_cells:
                # Mark obstacle cell
                self.grid[x_grid, y_grid] = 1.0
                
                # Mark free space along the ray
                steps = int(distance_cm / self.resolution)
                for step in range(1, steps):
                    x_step = self.vehicle_x + int(step * math.cos(rad))
                    y_step = self.vehicle_y + int(step * math.sin(rad))
                    if 0 <= x_step < self.grid_cells and 0 <= y_step < self.grid_cells:
                        # Free space gets lower value
                        self.grid[x_step, y_step] = 0.2
    
    def get_free_space_direction(self):
        """
        Find the largest contiguous free space
        Returns: (angle_to_free_space, clearance_cm)
        """
        # Simple heuristic: look at frontal arc (-30 to +30 degrees)
        best_angle = 0
        max_clearance = 0
        
        for angle in range(-30, 31, 5):
            clearance = self.check_clearance_at_angle(angle)
            if clearance > max_clearance:
                max_clearance = clearance
                best_angle = angle
        
        return best_angle, max_clearance
    
    def check_clearance_at_angle(self, angle_deg, max_check=100):
        """
        Check how far we can go in a given direction
        Returns: clearance in cm
        """
        rad = math.radians(angle_deg)
        
        for distance in range(10, max_check + 10, 10):
            x_rel = distance * math.cos(rad)
            y_rel = distance * math.sin(rad)
            
            x_grid = self.vehicle_x + int(x_rel / self.resolution)
            y_grid = self.vehicle_y + int(y_rel / self.resolution)
            
            if not (0 <= x_grid < self.grid_cells and 0 <= y_grid < self.grid_cells):
                return distance - 10
            
            if self.grid[x_grid, y_grid] > 0.7:  # Likely occupied
                return distance - 10
        
        return max_check
    
    def visualize_grid(self):
        """Simple ASCII visualization of occupancy grid"""
        for y in range(self.grid_cells):
            row = ""
            for x in range(self.grid_cells):
                if x == self.vehicle_x and y == self.vehicle_y:
                    row += "V"  # Vehicle
                elif self.grid[x, y] > 0.8:
                    row += "#"  # Occupied
                elif self.grid[x, y] < 0.3:
                    row += "."  # Free
                else:
                    row += "?"  # Unknown
            print(row)