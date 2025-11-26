import numpy as np
from .coordinate_transform import world_to_grid

def assign_cr_to_grid(grid, ts_position, cr_value, ship_domain, os_position, os_heading, grid_size, obs_distance):
    """
    Assign CR value to grid cells occupied by TS
    
    Args:
        grid: N×N numpy array (initialized with zeros)
        ts_position: TS position in world coordinates
        cr_value: Collision risk value (0~1)
        ship_domain: Ship domain radius (meters)
        ...
    """
    center = world_to_grid(ts_position, os_position, os_heading, grid_size, obs_distance)
    
    if center is None:
        return  # TS outside observation range
    
    grid_x, grid_y = center
    cell_size = obs_distance / grid_size
    
    # Calculate ship domain in grid cells
    domain_cells = int(np.ceil(ship_domain / cell_size))
    
    # Fill circular region around TS with CR value
    for dx in range(-domain_cells, domain_cells + 1):
        for dy in range(-domain_cells, domain_cells + 1):
            x = grid_x + dx
            y = grid_y + dy
            
            # Check bounds
            if 0 <= x < grid_size and 0 <= y < grid_size:
                # Check if within circular ship domain
                dist = np.sqrt(dx**2 + dy**2) * cell_size
                if dist <= ship_domain:
                    # Take maximum CR if multiple TS overlap
                    grid[y, x] = max(grid[y, x], cr_value)

def create_collision_risk_grid(os_position, os_heading, ts_list, grid_size=64, obs_distance=200):
    """
    Create collision risk grid (Chun et al. 2024 style)
    
    Args:
        os_position: [N, E] OS position
        os_heading: OS heading (degrees)
        ts_list: List of dicts with 'position', 'cr_value', 'ship_domain'
        grid_size: Grid dimension
        obs_distance: Observation range (meters)
    
    Returns:
        grid: (grid_size, grid_size) numpy array with CR values
    """
    # Initialize empty grid
    grid = np.zeros((grid_size, grid_size), dtype=np.float32)
    
    # Add each TS to grid
    for ts in ts_list:
        assign_cr_to_grid(
            grid=grid,
            ts_position=ts['position'],
            cr_value=ts['cr_value'],
            ship_domain=ts.get('ship_domain', 6.0),  # Default 6m
            os_position=os_position,
            os_heading=os_heading,
            grid_size=grid_size,
            obs_distance=obs_distance
        )
    
    return grid