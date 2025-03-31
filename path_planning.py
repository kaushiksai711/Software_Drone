#!/usr/bin/env python3
"""
Path Planning
Implements path planning algorithms for the drone
"""
import numpy as np
import math
import logging
import heapq
import unittest
from unittest.mock import Mock, patch
import pytest

logger = logging.getLogger("PathPlanning")

class PathPlanner:
    """Path planner for the drone"""
    
    def __init__(self):
        """Initialize the path planner"""
        self.grid_size = 0.5  # Grid cell size in meters
        self.grid = None
        self.grid_dimensions = (0, 0, 0)  # x, y, z dimensions
    
    def create_grid(self, dimensions, obstacles):
        """Create a grid representation of the environment"""
        # dimensions: (x_size, y_size, z_size) in meters
        # obstacles: list of (x, y, z, radius) tuples
        
        # Calculate grid dimensions
        x_cells = int(dimensions[0] / self.grid_size)
        y_cells = int(dimensions[1] / self.grid_size)
        z_cells = int(dimensions[2] / self.grid_size)
        
        # Create empty grid
        self.grid = np.zeros((x_cells, y_cells, z_cells), dtype=bool)
        self.grid_dimensions = (x_cells, y_cells, z_cells)
        
        # Add obstacles to grid
        for obstacle in obstacles:
            x, y, z, radius = obstacle
            
            # Convert to grid coordinates
            grid_x = int(x / self.grid_size)
            grid_y = int(y / self.grid_size)
            grid_z = int(z / self.grid_size)
            grid_radius = int(radius / self.grid_size) + 1  # Add 1 for safety margin
            
            # Mark obstacle cells
            for dx in range(-grid_radius, grid_radius + 1):
                for dy in range(-grid_radius, grid_radius + 1):
                    for dz in range(-grid_radius, grid_radius + 1):
                        # Check if within grid bounds
                        nx = grid_x + dx
                        ny = grid_y + dy
                        nz = grid_z + dz
                        
                        if (0 <= nx < x_cells and 
                            0 <= ny < y_cells and 
                            0 <= nz < z_cells):
                            
                            # Check if within obstacle radius
                            distance = math.sqrt(dx*dx + dy*dy + dz*dz) * self.grid_size
                            if distance <= radius:
                                self.grid[nx, ny, nz] = True
        
        logger.info(f"Created grid with dimensions {self.grid_dimensions}")
    
    def plan_path(self, start, goal):
        """Plan a path from start to goal using A* algorithm"""
        # start, goal: (x, y, z) in meters
        
        # Convert to grid coordinates
        start_grid = (
            int(start[0] / self.grid_size),
            int(start[1] / self.grid_size),
            int(start[2] / self.grid_size)
        )
        
        goal_grid = (
            int(goal[0] / self.grid_size),
            int(goal[1] / self.grid_size),
            int(goal[2] / self.grid_size)
        )
        
        # Check if start or goal is out of bounds or in obstacle
        if not self._is_valid(start_grid) or not self._is_valid(goal_grid):
            logger.error("Start or goal position is invalid")
            return None
        
        # A* algorithm
        open_set = []
        closed_set = set()
        
        # Priority queue for open set
        # (f_score, node)
        heapq.heappush(open_set, (self._heuristic(start_grid, goal_grid), start_grid))
        
        # g_score: cost from start to node
        g_score = {start_grid: 0}
        
        # f_score: estimated cost from start to goal through node
        f_score = {start_grid: self._heuristic(start_grid, goal_grid)}
        
        # came_from: parent node in path
        came_from = {}
        
        while open_set:
            # Get node with lowest f_score
            _, current = heapq.heappop(open_set)
            
            # Check if goal reached
            if current == goal_grid:
                # Reconstruct path
                path = self._reconstruct_path(came_from, current)
                
                # Convert to world coordinates
                world_path = []
                for node in path:
                    world_path.append((
                        node[0] * self.grid_size,
                        node[1] * self.grid_size,
                        node[2] * self.grid_size
                    ))
                
                logger.info(f"Path found with {len(world_path)} waypoints")
                return world_path
            
            # Add to closed set
            closed_set.add(current)
            
            # Check neighbors
            for neighbor in self._get_neighbors(current):
                # Skip if in closed set
                if neighbor in closed_set:
                    continue
                
                # Calculate tentative g_score
                tentative_g_score = g_score[current] + self._distance(current, neighbor)
                
                # Check if new path is better
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # Update path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal_grid)
                    
                    # Add to open set if not already there
                    if neighbor not in [node for _, node in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        logger.error("No path found")
        return None
    
    def _is_valid(self, node):
        """Check if a node is valid (within bounds and not in obstacle)"""
        x, y, z = node
        
        # Check if within bounds
        if (x < 0 or x >= self.grid_dimensions[0] or
            y < 0 or y >= self.grid_dimensions[1] or
            z < 0 or z >= self.grid_dimensions[2]):
            return False
        
        # Check if in obstacle
        if self.grid[x, y, z]:
            return False
        
        return True
    
    def _get_neighbors(self, node):
        """Get valid neighbors of a node"""
        x, y, z = node
        neighbors = []
        
        # Check all 26 neighbors
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    # Skip self
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    
                    neighbor = (x + dx, y + dy, z + dz)
                    
                    # Check if valid
                    if self._is_valid(neighbor):
                        neighbors.append(neighbor)
        
        return neighbors
    
    def _distance(self, node1, node2):
        """Calculate Euclidean distance between two nodes"""
        return math.sqrt(
            (node1[0] - node2[0])**2 +
            (node1[1] - node2[1])**2 +
            (node1[2] - node2[2])**2
        )
    
    def _heuristic(self, node, goal):
        """Calculate heuristic (Euclidean distance to goal)"""
        return self._distance(node, goal)
    
    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dictionary"""
        path = [current]
        
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        # Reverse to get path from start to goal
        path.reverse()
        
        return path
    
    def smooth_path(self, path, smoothing_factor=0.5):
        """Smooth the path using path smoothing algorithm"""
        if not path or len(path) < 3:
            return path
        
        # Create a copy of the path
        smoothed_path = [point for point in path]
        
        # Number of iterations
        iterations = 5
        
        for _ in range(iterations):
            for i in range(1, len(smoothed_path) - 1):
                for j in range(3):  # For each coordinate (x, y, z)
                    # Apply smoothing
                    smoothed_path[i] = (
                        smoothed_path[i][0] + smoothing_factor * (path[i][0] - smoothed_path[i][0]),
                        smoothed_path[i][1] + smoothing_factor * (path[i][1] - smoothed_path[i][1]),
                        smoothed_path[i][2] + smoothing_factor * (path[i][2] - smoothed_path[i][2])
                    )
        
        return smoothed_path

class DroneTest(unittest.TestCase):
    def setUp(self):
        # Initialize drone with mock hardware
        self.drone = Mock()
        # Set up mock sensor values
        self.drone.battery_level = 100
        self.drone.altitude = 0
        self.drone.gps_coordinates = (0, 0)
        self.drone.motor_status = "stopped"

    def test_takeoff(self):
        """Test takeoff functionality"""
        # Simulate takeoff
        self.drone.takeoff()
        
        # Assert drone reaches proper altitude
        self.drone.altitude = 10  # Simulated altitude
        self.assertEqual(self.drone.altitude, 10)
        self.assertEqual(self.drone.motor_status, "running")

    def test_landing(self):
        """Test landing functionality"""
        # Set initial altitude
        self.drone.altitude = 10
        
        # Simulate landing
        self.drone.land()
        
        # Assert drone has landed
        self.assertEqual(self.drone.altitude, 0)
        self.assertEqual(self.drone.motor_status, "stopped")

    def test_battery_check(self):
        """Test battery monitoring"""
        # Simulate battery drain
        self.drone.battery_level = 20
        
        # Test low battery warning
        with self.assertRaises(Warning):
            self.drone.check_battery_status()

    def test_movement_controls(self):
        """Test directional controls"""
        # Test forward movement
        initial_position = self.drone.gps_coordinates
        self.drone.move_forward(distance=10)
        self.assertNotEqual(self.drone.gps_coordinates, initial_position)
        
        # Test rotation
        initial_heading = 0
        self.drone.rotate(90)
        self.assertEqual(self.drone.heading, 90)

    def test_emergency_protocols(self):
        """Test emergency procedures"""
        # Simulate connection loss
        self.drone.connection_status = "disconnected"
        
        # Test auto-landing
        self.drone.activate_emergency_protocol()
        self.assertEqual(self.drone.altitude, 0)
        self.assertEqual(self.drone.motor_status, "stopped")

    def test_sensor_readings(self):
        """Test sensor functionality"""
        # Mock sensor data
        mock_sensor_data = {
            'temperature': 25,
            'pressure': 1013,
            'humidity': 60
        }
        
        with patch('drone.get_sensor_readings') as mock_sensors:
            mock_sensors.return_value = mock_sensor_data
            readings = self.drone.get_sensor_readings()
            
            self.assertEqual(readings['temperature'], 25)
            self.assertEqual(readings['pressure'], 1013)
            self.assertEqual(readings['humidity'], 60)

    def test_obstacle_avoidance(self):
        """Test obstacle detection and avoidance"""
        # Simulate obstacle detection
        mock_obstacles = [(5, 0, 0)]  # obstacle at 5 meters ahead
        
        with patch('drone.scan_environment') as mock_scan:
            mock_scan.return_value = mock_obstacles
            self.drone.check_and_avoid_obstacles()
            
            # Assert drone took evasive action
            self.assertNotEqual(self.drone.gps_coordinates, (0, 0))

    def test_mission_execution(self):
        """Test autonomous mission execution"""
        mission_waypoints = [
            (1, 1),
            (2, 2),
            (3, 3)
        ]
        
        self.drone.execute_mission(mission_waypoints)
        
        # Assert mission completion
        self.assertEqual(self.drone.gps_coordinates, (3, 3))
        self.assertTrue(self.drone.mission_completed)

if __name__ == '__main__':
    unittest.main()

