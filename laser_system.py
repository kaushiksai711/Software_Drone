#!/usr/bin/env python3
"""
Laser System
Handles the laser module for distance measurement and mapping
"""
import time
import logging
import threading
import numpy as np
import serial
import json
from collections import deque
from scipy import stats
import os

logger = logging.getLogger("LaserSystem")

class LaserSystem:
    """Interface to the laser module for distance measurement"""
    
    def __init__(self, port="/dev/ttyUSB1", baudrate=115200, config_file="config/laser_config.json"):
        """Initialize the laser system"""
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.is_running = False
        self.lock = threading.Lock()
        
        # Data storage with fixed size
        self.max_buffer_size = 360
        self.distances = deque(maxlen=self.max_buffer_size)
        self.angles = deque(maxlen=self.max_buffer_size)
        self.point_cloud = []
        
        # Performance monitoring
        self.last_read_time = 0
        self.read_count = 0
        self.error_count = 0
        self.processing_times = deque(maxlen=100)
        
        # Configuration
        self.config = self._load_config(config_file)
        
        # Connect to the laser module
        self._connect()
    
    def _load_config(self, config_file):
        """Load configuration from JSON file"""
        default_config = {
            "min_distance": 0.1,  # meters
            "max_distance": 30.0,  # meters
            "min_angle": 0,  # degrees
            "max_angle": 360,  # degrees
            "noise_threshold": 0.1,  # meters
            "outlier_threshold": 3.0,  # standard deviations
            "calibration_time": 2.0,  # seconds
            "reconnect_attempts": 3,
            "reconnect_delay": 1.0,  # seconds
            "checksum_enabled": True
        }
        
        try:
            if os.path.exists(config_file):
                with open(config_file, 'r') as f:
                    config = json.load(f)
                    # Update with defaults for missing values
                    for key, value in default_config.items():
                        if key not in config:
                            config[key] = value
                    return config
            else:
                logger.warning(f"Config file {config_file} not found, using defaults")
                return default_config
        except Exception as e:
            logger.error(f"Error loading config: {e}")
            return default_config
    
    def _connect(self):
        """Connect to the laser module via serial with retry mechanism"""
        attempts = 0
        while attempts < self.config['reconnect_attempts']:
            try:
                # Check if port exists before trying to connect
                if not os.path.exists(self.port) and not self.port.startswith('COM'):
                    logger.warning(f"Port {self.port} does not exist")
                    attempts += 1
                    if attempts < self.config['reconnect_attempts']:
                        logger.info(f"Waiting {self.config['reconnect_delay']}s before retrying...")
                        time.sleep(self.config['reconnect_delay'])
                    continue
                
                self.serial = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=1,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                logger.info(f"Connected to laser module on {self.port}")
                # Add a last_reading_time attribute for sensor failure detection
                self.last_reading_time = time.time()
                return
            except Exception as e:
                attempts += 1
                logger.error(f"Connection attempt {attempts} failed: {e}")
                if attempts < self.config['reconnect_attempts']:
                    time.sleep(self.config['reconnect_delay'])
        
        logger.error("Failed to connect to laser module after all attempts")
    
    def _validate_data(self, distance, angle):
        """Validate laser data"""
        if not (self.config['min_distance'] <= distance <= self.config['max_distance']):
            logger.warning(f"Invalid distance: {distance}m")
            return False
        
        if not (self.config['min_angle'] <= angle <= self.config['max_angle']):
            logger.warning(f"Invalid angle: {angle}°")
            return False
        
        return True
    
    def _calculate_checksum(self, data):
        """Calculate checksum for data validation"""
        return sum(ord(c) for c in data) & 0xFF
    
    def _read_laser_data(self):
        """Read data from the laser module with improved error handling"""
        if not self.serial or not self.serial.is_open:
            self._reconnect()
            return
        
        try:
            if self.serial.in_waiting > 0:
                start_time = time.time()
                
                # Read a line of data
                data = self.serial.readline().decode().strip()
                
                # Validate checksum if enabled
                if self.config['checksum_enabled'] and ':' in data:
                    data_part, checksum = data.split(':')
                    if int(checksum) != self._calculate_checksum(data_part):
                        logger.warning("Checksum mismatch")
                        self.error_count += 1
                        return
                else:
                    data_part = data
                
                # Parse the data
                if data_part.startswith("D"):
                    parts = data_part[2:].split(",")
                    if len(parts) >= 2:
                        try:
                            distance = float(parts[0])
                            angle = float(parts[1])
                            
                            # Validate data
                            if self._validate_data(distance, angle):
                                with self.lock:
                                    self.distances.append(distance)
                                    self.angles.append(angle)
                                
                                # Update performance metrics
                                self.read_count += 1
                                self.processing_times.append(time.time() - start_time)
                        except ValueError as e:
                            logger.error(f"Error parsing data: {e}")
                            self.error_count += 1
                
        except Exception as e:
            logger.error(f"Error reading laser data: {e}")
            self.error_count += 1
    
    def _reconnect(self):
        """Attempt to reconnect to the laser module"""
        logger.info("Attempting to reconnect to laser module")
        self._connect()
    
    def _filter_noise(self, distances, angles):
        """Filter noise from laser data"""
        if len(distances) < 3:
            return distances, angles
        
        # Convert to numpy arrays for processing
        dist_array = np.array(distances)
        angle_array = np.array(angles)
        
        # Remove statistical outliers
        z_scores = stats.zscore(dist_array)
        mask = abs(z_scores) < self.config['outlier_threshold']
        
        # Apply median filter for noise reduction
        filtered_dist = np.zeros_like(dist_array)
        window_size = 3
        for i in range(len(dist_array)):
            start = max(0, i - window_size // 2)
            end = min(len(dist_array), i + window_size // 2 + 1)
            filtered_dist[i] = np.median(dist_array[start:end])
        
        # Apply threshold for noise
        noise_mask = abs(filtered_dist - dist_array) < self.config['noise_threshold']
        final_mask = mask & noise_mask
        
        return filtered_dist[final_mask], angle_array[final_mask]
    
    def _process_laser_data(self):
        """Process the laser data with noise filtering"""
        with self.lock:
            if len(self.distances) < 3:
                return
            
            # Filter noise
            filtered_distances, filtered_angles = self._filter_noise(
                list(self.distances),
                list(self.angles)
            )
            
            # Convert polar coordinates to Cartesian
            point_cloud = []
            for distance, angle in zip(filtered_distances, filtered_angles):
                angle_rad = np.radians(angle)
                x = distance * np.cos(angle_rad)
                y = distance * np.sin(angle_rad)
                point_cloud.append((x, y))
            
            self.point_cloud = point_cloud
    
    def get_performance_metrics(self):
        """Get system performance metrics"""
        with self.lock:
            return {
                'read_count': self.read_count,
                'error_count': self.error_count,
                'error_rate': self.error_count / max(1, self.read_count),
                'avg_processing_time': np.mean(self.processing_times) if self.processing_times else 0,
                'buffer_size': len(self.distances)
            }
    
    def get_distances(self):
        """Get the latest distance measurements"""
        with self.lock:
            return list(self.distances)
    
    def get_point_cloud(self):
        """Get the latest point cloud"""
        with self.lock:
            return self.point_cloud.copy()
    
    def get_nearest_obstacle_distance(self):
        """Get the distance to the nearest obstacle"""
        with self.lock:
            if not self.distances:
                return float('inf')
            
            # Filter out invalid measurements
            valid_distances = [d for d in self.distances 
                             if self.config['min_distance'] <= d <= self.config['max_distance']]
            
            return min(valid_distances) if valid_distances else float('inf')
    
    def calibrate(self):
        """Calibrate the laser system with improved error handling"""
        logger.info("Calibrating laser system")
        
        if not self.serial or not self.serial.is_open:
            logger.warning("Cannot calibrate laser system: not connected")
            return False
        
        try:
            # Send calibration command
            self.serial.write(b"CALIBRATE\n")
            
            # Wait for calibration to complete
            time.sleep(self.config['calibration_time'])
            
            # Clear existing data
            with self.lock:
                self.distances.clear()
                self.angles.clear()
                self.point_cloud = []
            
            # Reset error counters
            self.error_count = 0
            self.read_count = 0
            
            logger.info("Laser system calibration completed")
            return True
            
        except Exception as e:
            logger.error(f"Error during calibration: {e}")
            return False 