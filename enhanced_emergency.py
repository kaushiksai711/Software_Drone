#!/usr/bin/env python3
"""
Enhanced Emergency System
Additional safety measures for emergency situations
"""
import time
import logging
import threading
import math
import numpy as np

logger = logging.getLogger("EnhancedEmergency")

class EnhancedEmergencySystem:
    """Enhanced emergency system with additional safety measures"""
    
    def __init__(self, drone_controller):
        """Initialize the enhanced emergency system"""
        self.drone_controller = drone_controller
        self.is_running = False
        
        # Geofencing parameters
        self.geofence_enabled = True
        self.geofence_center = (0, 0)  # x, y center in meters
        self.geofence_radius = 50.0    # radius in meters
        
        # Wind detection parameters
        self.wind_detection_enabled = True
        self.wind_samples = []
        self.max_wind_samples = 20
        self.wind_threshold = 5.0      # m/s
        
        # Sensor failure detection
        self.sensor_failure_detection_enabled = True
        self.last_sensor_readings = {}
        self.sensor_timeout = 2.0      # seconds
        
        # Vibration detection
        self.vibration_detection_enabled = True
        self.vibration_samples = []
        self.max_vibration_samples = 50
        self.vibration_threshold = 2.0  # g
        
        # Failsafe actions
        self.failsafe_actions = {
            'geofence_breach': 'RETURN_TO_BASE',
            'high_wind': 'LAND',
            'sensor_failure': 'LAND',
            'high_vibration': 'LAND'
        }
    
    def start(self):
        """Start the enhanced emergency system"""
        if self.is_running:
            return
        
        self.is_running = True
        self.emergency_thread = threading.Thread(target=self._emergency_loop)
        self.emergency_thread.daemon = True
        self.emergency_thread.start()
        
        logger.info("Enhanced emergency system started")
    
    def stop(self):
        """Stop the enhanced emergency system"""
        if not self.is_running:
            return
        
        self.is_running = False
        
        # Wait for emergency thread to finish
        if hasattr(self, 'emergency_thread') and self.emergency_thread.is_alive():
            self.emergency_thread.join(timeout=2.0)
        
        logger.info("Enhanced emergency system stopped")
    
    def _emergency_loop(self):
        """Main emergency monitoring loop"""
        while self.is_running:
            try:
                # Check geofence
                if self.geofence_enabled:
                    self._check_geofence()
                
                # Check wind
                if self.wind_detection_enabled:
                    self._check_wind()
                
                # Check sensor failures
                if self.sensor_failure_detection_enabled:
                    self._check_sensor_failures()
                
                # Check vibration
                if self.vibration_detection_enabled:
                    self._check_vibration()
                
                # Sleep to maintain monitoring frequency
                time.sleep(0.5)  # 2Hz monitoring rate
                
            except Exception as e:
                logger.error(f"Error in enhanced emergency monitoring: {e}")
                time.sleep(1)  # Wait before retrying
    
    def _check_geofence(self):
        """Check if drone is within geofence"""
        # Get current position
        current_position = self.drone_controller.navigation.get_current_position()
        
        if current_position:
            # Calculate distance from geofence center
            dx = current_position[0] - self.geofence_center[0]
            dy = current_position[1] - self.geofence_center[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if outside geofence
            if distance > self.geofence_radius:
                logger.warning(f"GEOFENCE BREACH: {distance:.2f}m from center (limit: {self.geofence_radius:.2f}m)")
                self._activate_failsafe('geofence_breach')
    
    def _check_wind(self):
        """Check for high wind conditions"""
        # In a real system, this would use sensor data to estimate wind
        # For this example, we'll use a simple model based on control inputs vs. actual movement
        
        # Get current velocity
        if hasattr(self.drone_controller.navigation, 'current_velocity'):
            velocity = self.drone_controller.navigation.current_velocity
            
            # Get control inputs
            if hasattr(self.drone_controller.flight_controller, 'pitch') and hasattr(self.drone_controller.flight_controller, 'roll'):
                pitch = self.drone_controller.flight_controller.pitch
                roll = self.drone_controller.flight_controller.roll
                
                # Estimate expected velocity based on control inputs
                expected_vx = pitch / 500.0 * 5.0  # Max 5 m/s at full pitch
                expected_vy = roll / 500.0 * 5.0   # Max 5 m/s at full roll
                
                # Calculate difference (potential wind effect)
                if hasattr(velocity, '__len__') and len(velocity) >= 2:
                    wind_vx = velocity[0] - expected_vx
                    wind_vy = velocity[1] - expected_vy
                    
                    # Calculate wind magnitude
                    wind_magnitude = math.sqrt(wind_vx*wind_vx + wind_vy*wind_vy)
                    
                    # Add to samples
                    self.wind_samples.append(wind_magnitude)
                    
                    # Keep only recent samples
                    if len(self.wind_samples) > self.max_wind_samples:
                        self.wind_samples.pop(0)
                    
                    # Calculate average wind
                    if self.wind_samples:
                        avg_wind = sum(self.wind_samples) / len(self.wind_samples)
                        
                        # Check if wind is too high
                        if avg_wind > self.wind_threshold:
                            logger.warning(f"HIGH WIND DETECTED: {avg_wind:.2f} m/s (threshold: {self.wind_threshold:.2f} m/s)")
                            self._activate_failsafe('high_wind')
    
    def _check_sensor_failures(self):
        """Check for sensor failures"""
        current_time = time.time()
        
        # Check IMU
        if hasattr(self.drone_controller.flight_controller, 'last_imu_reading_time'):
            last_imu_time = self.drone_controller.flight_controller.last_imu_reading_time
            if current_time - last_imu_time > self.sensor_timeout:
                logger.warning(f"IMU SENSOR FAILURE: No data for {current_time - last_imu_time:.2f}s")
                self._activate_failsafe('sensor_failure')
        
        # Check barometer
        if hasattr(self.drone_controller.flight_controller, 'last_baro_reading_time'):
            last_baro_time = self.drone_controller.flight_controller.last_baro_reading_time
            if current_time - last_baro_time > self.sensor_timeout:
                logger.warning(f"BAROMETER SENSOR FAILURE: No data for {current_time - last_baro_time:.2f}s")
                self._activate_failsafe('sensor_failure')
        
        # Check laser
        if hasattr(self.drone_controller.laser_system, 'last_reading_time'):
            last_laser_time = self.drone_controller.laser_system.last_reading_time
            if current_time - last_laser_time > self.sensor_timeout:
                logger.warning(f"LASER SENSOR FAILURE: No data for {current_time - last_laser_time:.2f}s")
                self._activate_failsafe('sensor_failure')
    
    def _check_vibration(self):
        """Check for excessive vibration"""
        # In a real system, this would use accelerometer data
        # For this example, we'll use a simple model
        
        if hasattr(self.drone_controller.flight_controller, 'acceleration'):
            accel = self.drone_controller.flight_controller.acceleration
            
            if hasattr(accel, '__len__') and len(accel) >= 3:
                # Calculate vibration magnitude (remove gravity component)
                vibration = math.sqrt(accel[0]*accel[0] + accel[1]*accel[1] + (accel[2]-9.81)*(accel[2]-9.81))
                
                # Add to samples
                self.vibration_samples.append(vibration)
                
                # Keep only recent samples
                if len(self.vibration_samples) > self.max_vibration_samples:
                    self.vibration_samples.pop(0)
                
                # Calculate average vibration
                if self.vibration_samples:
                    avg_vibration = sum(self.vibration_samples) / len(self.vibration_samples)
                    
                    # Check if vibration is too high
                    if avg_vibration > self.vibration_threshold:
                        logger.warning(f"HIGH VIBRATION DETECTED: {avg_vibration:.2f} g (threshold: {self.vibration_threshold:.2f} g)")
                        self._activate_failsafe('high_vibration')
    
    def _activate_failsafe(self, failsafe_type):
        """Activate failsafe action"""
        # Get failsafe action
        action = self.failsafe_actions.get(failsafe_type)
        
        if not action:
            logger.error(f"No failsafe action defined for {failsafe_type}")
            return
        
        logger.critical(f"ACTIVATING FAILSAFE ({failsafe_type}): {action}")
        
        # Execute failsafe action
        if action == 'LAND':
            self.drone_controller.land_at_current_position()
        elif action == 'RETURN_TO_BASE':
            if self.drone_controller.home_position:
                self.drone_controller.set_mode(self.drone_controller.DroneMode.RETURN_TO_BASE)
            else:
                logger.warning("Home position not set, landing instead")
                self.drone_controller.land_at_current_position()
        elif action == 'EMERGENCY':
            self.drone_controller.set_mode(self.drone_controller.DroneMode.EMERGENCY)
    
    def set_geofence(self, center, radius):
        """Set geofence parameters"""
        self.geofence_center = center
        self.geofence_radius = radius
        logger.info(f"Geofence set: center={center}, radius={radius}m")
    
    def enable_geofence(self, enabled=True):
        """Enable or disable geofence"""
        self.geofence_enabled = enabled
        logger.info(f"Geofence {'enabled' if enabled else 'disabled'}")
    
    def set_wind_threshold(self, threshold):
        """Set wind threshold"""
        self.wind_threshold = threshold
        logger.info(f"Wind threshold set to {threshold} m/s")
    
    def enable_wind_detection(self, enabled=True):
        """Enable or disable wind detection"""
        self.wind_detection_enabled = enabled
        logger.info(f"Wind detection {'enabled' if enabled else 'disabled'}")
    
    def set_vibration_threshold(self, threshold):
        """Set vibration threshold"""
        self.vibration_threshold = threshold
        logger.info(f"Vibration threshold set to {threshold} g")
    
    def enable_vibration_detection(self, enabled=True):
        """Enable or disable vibration detection"""
        self.vibration_detection_enabled = enabled
        logger.info(f"Vibration detection {'enabled' if enabled else 'disabled'}")
    
    def set_failsafe_action(self, failsafe_type, action):
        """Set failsafe action for a specific failsafe type"""
        if action in ['LAND', 'RETURN_TO_BASE', 'EMERGENCY']:
            self.failsafe_actions[failsafe_type] = action
            logger.info(f"Failsafe action for {failsafe_type} set to {action}")
        else:
            logger.error(f"Invalid failsafe action: {action}")

