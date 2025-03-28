#!/usr/bin/env python3
"""
Navigation System
Handles path planning, obstacle avoidance, and movement control
"""
import time
import logging
import threading
import numpy as np
from enum import Enum
import math

logger = logging.getLogger("Navigation")

class NavigationState(Enum):
    """Navigation system states"""
    IDLE = 0
    HOVERING = 1
    TRAVERSING = 2
    LANDING = 3
    RETURNING = 4

class NavigationSystem:
    """Navigation system for the drone"""
    
    def __init__(self, camera_system, laser_system):
        """Initialize the navigation system"""
        self.camera_system = camera_system
        self.laser_system = laser_system
        self.is_running = False
        self.state = NavigationState.IDLE
        self.current_position = (0, 0, 0)  # x, y, z in meters
        self.current_orientation = (0, 0, 0)  # roll, pitch, yaw in degrees
        self.target_position = None
        self.path = []
        self.path_index = 0
        self.hover_height = 1.5  # meters
        self.is_landed = False
        self.home_position = None
        self.lock = threading.Lock()
        
        # PID controllers for position control
        self.pid_x = PIDController(0.5, 0.1, 0.2)
        self.pid_y = PIDController(0.5, 0.1, 0.2)
        self.pid_z = PIDController(0.8, 0.2, 0.3)
        self.pid_yaw = PIDController(0.3, 0.05, 0.1)
    
    def start(self):
        """Start the navigation system"""
        if self.is_running:
            return
        
        self.is_running = True
        self.nav_thread = threading.Thread(target=self._navigation_loop)
        self.nav_thread.daemon = True
        self.nav_thread.start()
        
        logger.info("Navigation system started")
    
    def stop(self):
        """Stop the navigation system"""
        if not self.is_running:
            return
        
        self.is_running = False
        
        # Wait for navigation thread to finish
        if hasattr(self, 'nav_thread') and self.nav_thread.is_alive():
            self.nav_thread.join(timeout=2.0)
        
        logger.info("Navigation system stopped")
    
    def _navigation_loop(self):
        """Main navigation processing loop"""
        while self.is_running:
            try:
                # Update current position and orientation
                self._update_position()
                
                # Process current navigation state
                if self.state == NavigationState.HOVERING:
                    self._process_hovering()
                elif self.state == NavigationState.TRAVERSING:
                    self._process_traversing()
                elif self.state == NavigationState.LANDING:
                    self._process_landing()
                elif self.state == NavigationState.RETURNING:
                    self._process_returning()
                
                # Maintain processing rate
                time.sleep(0.05)  # 20Hz processing rate
                
            except Exception as e:
                logger.error(f"Error in navigation processing: {e}")
                time.sleep(1)  # Wait before retrying
    
    def _update_position(self):
        """Update the current position and orientation based on sensors"""
        # In a real system, this would use sensor fusion from IMU, GPS, etc.
        # For this example, we'll simulate position updates
        
        # TODO: Implement actual position estimation using sensor fusion
        pass
    
    def _process_hovering(self):
        """Process hovering state"""
        with self.lock:
            # Get current height
            current_height = self.current_position[2]
            
            # Calculate height error
            height_error = self.hover_height - current_height
            
            # Use PID controller to adjust height
            z_control = self.pid_z.update(height_error)
            
            # Apply control to maintain hover
            # This would send commands to the flight controller
            logger.debug(f"Hover control: z={z_control:.2f}, height={current_height:.2f}m, target={self.hover_height:.2f}m")
    
    def _process_traversing(self):
        """Process traversing state"""
        with self.lock:
            if not self.path or self.path_index >= len(self.path):
                logger.info("Path completed or empty")
                self.state = NavigationState.HOVERING
                return
            
            # Get current waypoint
            waypoint = self.path[self.path_index]
            
            # Calculate distance to waypoint
            dx = waypoint[0] - self.current_position[0]
            dy = waypoint[1] - self.current_position[1]
            dz = waypoint[2] - self.current_position[2]
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Check if waypoint reached
            if distance < 0.2:  # 20cm threshold
                logger.info(f"Waypoint {self.path_index} reached")
                self.path_index += 1
                
                # Check if path completed
                if self.path_index >= len(self.path):
                    logger.info("Path completed")
                    self.state = NavigationState.HOVERING
                
                return
            
            # Check for obstacles
            if self._check_obstacles():
                # Replan path to avoid obstacles
                self._replan_path()
            
            # Use PID controllers to move towards waypoint
            x_control = self.pid_x.update(dx)
            y_control = self.pid_y.update(dy)
            z_control = self.pid_z.update(dz)
            
            # Calculate desired yaw (heading towards waypoint)
            desired_yaw = math.degrees(math.atan2(dy, dx))
            yaw_error = self._normalize_angle(desired_yaw - self.current_orientation[2])
            yaw_control = self.pid_yaw.update(yaw_error)
            
            # Apply control to move towards waypoint
            # This would send commands to the flight controller
            logger.debug(f"Traverse control: x={x_control:.2f}, y={y_control:.2f}, z={z_control:.2f}, yaw={yaw_control:.2f}")
    
    def _process_landing(self):
        """Process landing state"""
        with self.lock:
            # Get current height
            current_height = self.current_position[2]
            
            # If already landed, do nothing
            if self.is_landed:
                return
            
            # If landing target is set, move towards it
            if self.target_position:
                dx = self.target_position[0] - self.current_position[0]
                dy = self.target_position[1] - self.current_position[1]
                
                # Use PID controllers to move towards landing target
                x_control = self.pid_x.update(dx)
                y_control = self.pid_y.update(dy)
                
                # Apply control to move towards landing target
                # This would send commands to the flight controller
                logger.debug(f"Landing position control: x={x_control:.2f}, y={y_control:.2f}")
            
            # Calculate descent rate based on height
            # Slower descent as we get closer to the ground
            if current_height > 1.0:
                descent_rate = -0.5  # 0.5 m/s
            elif current_height > 0.5:
                descent_rate = -0.3  # 0.3 m/s
            elif current_height > 0.2:
                descent_rate = -0.1  # 0.1 m/s
            else:
                descent_rate = -0.05  # 0.05 m/s
            
            # Use PID controller to control descent
            z_control = descent_rate
            
            # Apply control for descent
            # This would send commands to the flight controller
            logger.debug(f"Landing descent control: z={z_control:.2f}, height={current_height:.2f}m")
            
            # Check if landed
            if current_height < 0.05:  # 5cm threshold
                logger.info("Landing completed")
                self.is_landed = True
                
                # Disarm motors
                # This would send commands to the flight controller
    
    def _process_returning(self):
        """Process returning to home state"""
        with self.lock:
            if not self.home_position:
                logger.warning("Home position not set, cannot return")
                self.state = NavigationState.HOVERING
                return
            
            # Calculate distance to home
            dx = self.home_position[0] - self.current_position[0]
            dy = self.home_position[1] - self.current_position[1]
            dz = self.hover_height - self.current_position[2]  # Maintain hover height during return
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if home reached
            if distance < 0.5:  # 50cm threshold
                logger.info("Home position reached")
                self.state = NavigationState.HOVERING
                return
            
            # Check for obstacles
            if self._check_obstacles():
                # Replan path to avoid obstacles
                self._replan_path()
            
            # Use PID controllers to move towards home
            x_control = self.pid_x.update(dx)
            y_control = self.pid_y.update(dy)
            z_control = self.pid_z.update(dz)
            
            # Calculate desired yaw (heading towards home)
            desired_yaw = math.degrees(math.atan2(dy, dx))
            yaw_error = self._normalize_angle(desired_yaw - self.current_orientation[2])
            yaw_control = self.pid_yaw.update(yaw_error)
            
            # Apply control to move towards home
            # This would send commands to the flight controller
            logger.debug(f"Return control: x={x_control:.2f}, y={y_control:.2f}, z={z_control:.2f}, yaw={yaw_control:.2f}")
    
    def _check_obstacles(self):
        """Check for obstacles in the path"""
        # Get obstacles from camera and laser
        camera_obstacles = self.camera_system.get_obstacles()
        
        # Get nearest obstacle distance from laser
        nearest_distance = self.laser_system.get_nearest_obstacle_distance()
        
        # Check if any obstacle is too close
        if nearest_distance < 1.0:  # 1 meter threshold
            logger.warning(f"Obstacle detected at {nearest_distance:.2f}m")
            return True
        
        # Check camera obstacles
        for obstacle in camera_obstacles:
            # TODO: Convert camera coordinates to world coordinates
            # and check if obstacle is in the path
            pass
        
        return False
    
    def _replan_path(self):
        """Replan path to avoid obstacles"""
        logger.info("Replanning path to avoid obstacles")
        
        # Get current position and target
        current = self.current_position
        
        if self.state == NavigationState.TRAVERSING:
            target = self.path[self.path_index]
        elif self.state == NavigationState.RETURNING:
            target = self.home_position
        else:
            return
        
        # Get obstacles
        obstacles = self.camera_system.get_obstacles()
        
        # Simple obstacle avoidance: move up and around
        # In a real system, this would use a more sophisticated algorithm
        
        # Create a new path
        new_path = []
        
        # Add waypoint above current position
        new_path.append((current[0], current[1], current[2] + 1.0))
        
        # Add waypoint to the side of the direct path
        mid_x = (current[0] + target[0]) / 2
        mid_y = (current[1] + target[1]) / 2
        
        # Move perpendicular to the path
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        length = math.sqrt(dx*dx + dy*dy)
        
        if length > 0:
            # Calculate perpendicular direction
            perp_x = -dy / length
            perp_y = dx / length
            
            # Add waypoint to the side
            new_path.append((mid_x + perp_x * 2.0, mid_y + perp_y * 2.0, current[2] + 1.0))
        
        # Add target waypoint
        new_path.append(target)
        
        # Update path
        with self.lock:
            self.path = new_path
            self.path_index = 0
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-180, 180] degrees"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def start_hovering(self, height):
        """Start hovering at the specified height"""
        with self.lock:
            logger.info(f"Starting hover at {height}m")
            self.hover_height = height
            self.state = NavigationState.HOVERING
            self.is_landed = False
    
    def start_landing(self):
        """Start landing procedure"""
        with self.lock:
            logger.info("Starting landing procedure")
            self.state = NavigationState.LANDING
            self.is_landed = False
    
    def return_to_base(self, home_position):
        """Start returning to base"""
        with self.lock:
            logger.info(f"Returning to base at {home_position}")
            self.home_position = home_position
            self.state = NavigationState.RETURNING
    
    def set_path(self, path):
        """Set a new path to follow"""
        with self.lock:
            logger.info(f"Setting new path with {len(path)} waypoints")
            self.path = path
            self.path_index = 0
            self.state = NavigationState.TRAVERSING
    
    def set_landing_target(self, target_position):
        """Set the landing target position"""
        with self.lock:
            logger.info(f"Setting landing target to {target_position}")
            self.target_position = target_position
    
    def get_current_height(self):
        """Get the current height"""
        return self.current_position[2]
    
    def get_current_position(self):
        """Get the current position"""
        return self.current_position
    
    def adjust_height(self, height):
        """Adjust the hover height"""
        with self.lock:
            logger.info(f"Adjusting hover height to {height}m")
            self.hover_height = height
    
    def is_path_completed(self):
        """Check if the current path is completed"""
        with self.lock:
            return self.state != NavigationState.TRAVERSING or (self.path_index >= len(self.path))
    
    def is_landed(self):
        """Check if the drone is landed"""
        return self.is_landed
    
    def get_mission_status(self):
        """Get the current mission status"""
        if self.state == NavigationState.IDLE:
            return "idle"
        elif self.state == NavigationState.HOVERING:
            return "hovering"
        elif self.state == NavigationState.TRAVERSING:
            if self.is_path_completed():
                return "completed"
            else:
                return "traversing"
        elif self.state == NavigationState.LANDING:
            if self.is_landed:
                return "landed"
            else:
                return "landing"
        elif self.state == NavigationState.RETURNING:
            return "returning"


class PIDController:
    """PID controller for position control"""
    
    def __init__(self, kp, ki, kd):
        """Initialize the PID controller"""
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def update(self, error):
        """Update the PID controller with a new error value"""
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Ensure dt is not too small
        if dt < 0.001:
            dt = 0.001
        
        # Calculate derivative
        derivative = (error - self.prev_error) / dt
        
        # Update integral
        self.integral += error * dt
        
        # Anti-windup: limit integral
        self.integral = max(-10, min(10, self.integral))
        
        # Calculate control output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Update previous error
        self.prev_error = error
        
        return output
    
    def reset(self):
        """Reset the PID controller"""
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

