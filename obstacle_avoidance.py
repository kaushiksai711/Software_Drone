#!/usr/bin/env python3
"""
Obstacle Avoidance
Implements obstacle avoidance algorithms
"""
import numpy as np
import math
import logging

logger = logging.getLogger("ObstacleAvoidance")

class ObstacleAvoidance:
    """Obstacle avoidance for the drone"""
    
    def __init__(self):
        """Initialize the obstacle avoidance system"""
        # Minimum safe distance to obstacles (in meters)
        self.safe_distance = 1.0
        
        # Maximum detection range (in meters)
        self.max_range = 5.0
        
        # Repulsive force parameters
        self.repulsive_gain = 1.0
        self.attractive_gain = 0.5
    
    def avoid_obstacles(self, current_position, target_position, obstacles):
        """Calculate avoidance vector based on obstacles"""
        # current_position, target_position: (x, y, z) in meters
        # obstacles: list of (x, y, z, radius) tuples
        
        # Initialize avoidance vector
        avoidance_vector = np.zeros(3)
        
        # Calculate attractive force towards target
        attractive_force = self._calculate_attractive_force(current_position, target_position)
        
        # Calculate repulsive force from obstacles
        repulsive_force = self._calculate_repulsive_force(current_position, obstacles)
        
        # Combine forces
        avoidance_vector = attractive_force + repulsive_force
        
        # Normalize avoidance vector
        magnitude = np.linalg.norm(avoidance_vector)
        if magnitude > 0:
            avoidance_vector = avoidance_vector / magnitude
        
        return avoidance_vector
    
    def _calculate_attractive_force(self, current_position, target_position):
        """Calculate attractive force towards target"""
        # Convert to numpy arrays
        current = np.array(current_position)
        target = np.array(target_position)
        
        # Calculate direction vector
        direction = target - current
        
        # Calculate distance
        distance = np.linalg.norm(direction)
        
        # Normalize direction
        if distance > 0:
            direction = direction / distance
        
        # Calculate attractive force
        attractive_force = self.attractive_gain * direction
        
        return attractive_force
    
    def _calculate_repulsive_force(self, current_position, obstacles):
        """Calculate repulsive force from obstacles"""
        # Convert to numpy array
        current = np.array(current_position)
        
        # Initialize repulsive force
        repulsive_force = np.zeros(3)
        
        # Process each obstacle
        for obstacle in obstacles:
            # Extract obstacle position and radius
            obstacle_position = np.array(obstacle[:3])
            obstacle_radius = obstacle[3]
            
            # Calculate direction vector from obstacle to drone
            direction = current - obstacle_position
            
            # Calculate distance
            distance = np.linalg.norm(direction)
            
            # Skip if obstacle is too far
            if distance > self.max_range:
                continue
            
            # Calculate actual distance (accounting for obstacle radius)
            actual_distance = distance - obstacle_radius
            
            # Skip if actual distance is negative (inside obstacle) or too large
            if actual_distance <= 0 or actual_distance > self.max_range:
                continue
            
            # Normalize direction
            if distance > 0:
                direction = direction / distance
            
            # Calculate repulsive force (inversely proportional to distance)
            # Force increases as distance decreases
            force_magnitude = self.repulsive_gain * (1.0 / actual_distance - 1.0 / self.max_range)
            
            # Ensure force is positive
            if force_magnitude > 0:
                # Add to total repulsive force
                repulsive_force += force_magnitude * direction
        
        return repulsive_force
    
    def is_path_clear(self, start, end, obstacles):
        """Check if path between start and end is clear of obstacles"""
        # start, end: (x, y, z) in meters
        # obstacles: list of (x, y, z, radius) tuples
        
        # Convert to numpy arrays
        start_point = np.array(start)
        end_point = np.array(end)
        
        # Calculate direction vector
        direction = end_point - start_point
        
        # Calculate distance
        distance = np.linalg.norm(direction)
        
        # Normalize direction
        if distance > 0:
            direction = direction / distance
        
        # Number of steps to check
        steps = int(distance / 0.1) + 1  # Check every 10cm
        
        # Check each point along the path
        for i in range(steps + 1):
            # Calculate point
            t = i / steps
            point = start_point + t * direction * distance
            
            # Check if point is too close to any obstacle
            for obstacle in obstacles:
                obstacle_position = np.array(obstacle[:3])
                obstacle_radius = obstacle[3]
                
                # Calculate distance to obstacle
                obstacle_distance = np.linalg.norm(point - obstacle_position)
                
                # Check if too close
                if obstacle_distance < obstacle_radius + self.safe_distance:
                    return False
        
        # Path is clear
        return True

