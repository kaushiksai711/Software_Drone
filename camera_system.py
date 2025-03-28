#!/usr/bin/env python3
"""
Camera System
Handles the ESP32 camera for visual detection and navigation
"""
import time
import logging
import threading
import cv2
import numpy as np
import requests
from io import BytesIO
from PIL import Image
import json
import os
from collections import deque
from scipy.spatial.transform import Rotation

logger = logging.getLogger("CameraSystem")

class CameraSystem:
    """Interface to the ESP32 camera module"""
    
    def __init__(self, camera_url="http://esp32-cam.local:80/stream", config_file="config/camera_config.json"):
        """Initialize the camera system"""
        self.camera_url = camera_url
        self.is_running = False
        self.frame = None
        self.frame_lock = threading.Lock()
        self.last_frame_time = 0
        self.box_detected = False
        self.box_position = None
        self.obstacles = []
        
        # Performance monitoring
        self.frame_count = 0
        self.processing_times = deque(maxlen=100)
        self.error_count = 0
        
        # Sensor fusion
        self.laser_system = None
        self.imu_data = None
        self.last_sync_time = 0
        self.sync_interval = 0.1  # 100ms sync interval
        
        # Calibration data
        self.calibration_data = None
        self.distortion_coeffs = None
        self.camera_matrix = None
        
        # Configuration
        self.config = self._load_config(config_file)
        
        # Initialize OpenCV for image processing
        self.initialize_cv()
    
    def initialize_cv(self):
        """Initialize OpenCV components"""
        try:
            # Create windows for debugging
            cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Processed Frame', cv2.WINDOW_NORMAL)
            logger.info("OpenCV windows initialized")
        except Exception as e:
            logger.error(f"Error initializing OpenCV: {e}")
    
    def start(self):
        """Start the camera system"""
        if self.is_running:
            return
        
        self.is_running = True
        self.camera_thread = threading.Thread(target=self._camera_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        
        logger.info("Camera system started")
    
    def stop(self):
        """Stop the camera system"""
        if not self.is_running:
            return
        
        self.is_running = False
        
        # Wait for camera thread to finish
        if hasattr(self, 'camera_thread') and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2.0)
        
        # Close OpenCV windows
        cv2.destroyAllWindows()
        
        logger.info("Camera system stopped")
    
    def _camera_loop(self):
        """Main camera processing loop"""
        while self.is_running:
            try:
                # Capture frame
                frame = self._capture_frame()
                if frame is not None:
                    # Process frame
                    self._process_frame(frame)
                    
                    # Update frame count
                    self.frame_count += 1
                    
                    # Synchronize with other sensors
                    self._synchronize_sensors()
                
                # Maintain processing rate
                time.sleep(0.05)  # 20Hz processing rate
                
            except Exception as e:
                logger.error(f"Error in camera loop: {e}")
                self.error_count += 1
                time.sleep(1)  # Wait before retrying
    
    def _capture_frame(self):
        """Capture a frame from the ESP32 camera"""
        try:
            # Get image from ESP32
            response = requests.get(self.camera_url, timeout=1.0)
            if response.status_code == 200:
                # Convert response to image
                image = Image.open(BytesIO(response.content))
                
                # Convert to OpenCV format
                frame = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
                
                # Apply calibration if available
                if self.calibration_data and self.distortion_coeffs is not None:
                    frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coeffs)
                
                return frame
            else:
                logger.error(f"Failed to get image from camera: {response.status_code}")
                return None
                
        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return None
    
    def _process_frame(self, frame):
        """Process the captured frame"""
        try:
            # Store frame
            with self.frame_lock:
                self.frame = frame
            
            # Detect obstacles
            self._detect_obstacles_from_camera(frame)
            
            # Update performance metrics
            self.processing_times.append(time.time() - self.last_frame_time)
            self.last_frame_time = time.time()
            
        except Exception as e:
            logger.error(f"Error processing frame: {e}")
    
    def get_frame(self):
        """Get the latest frame"""
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None
    
    def is_box_detected(self):
        """Check if a landing box is detected"""
        return self.box_detected
    
    def get_box_position(self):
        """Get the position of the detected landing box"""
        return self.box_position
    
    def get_obstacles(self):
        """Get the list of detected obstacles"""
        return self.obstacles.copy()
    
    def _load_config(self, config_file):
        """Load configuration from JSON file"""
        default_config = {
            "min_obstacle_area": 200,  # pixels
            "min_box_area": 500,  # pixels
            "calibration_grid_size": (9, 6),  # chessboard size
            "calibration_square_size": 0.025,  # meters
            "max_calibration_images": 20,
            "min_calibration_images": 10,
            "calibration_quality_threshold": 0.8,
            "sync_timeout": 0.5,  # seconds
            "max_obstacle_distance": 5.0,  # meters
            "min_obstacle_distance": 0.3,  # meters
            "fusion_weight_camera": 0.7,
            "fusion_weight_laser": 0.3
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
    
    def set_laser_system(self, laser_system):
        """Set the laser system for sensor fusion"""
        self.laser_system = laser_system
        logger.info("Laser system connected for sensor fusion")
    
    def update_imu_data(self, imu_data):
        """Update IMU data for sensor fusion"""
        self.imu_data = imu_data
        self.last_sync_time = time.time()
    
    def _synchronize_sensors(self):
        """Synchronize camera data with other sensors"""
        current_time = time.time()
        
        # Check if we have recent IMU data
        if self.imu_data and (current_time - self.last_sync_time) < self.config['sync_timeout']:
            # Get laser data if available
            laser_data = None
            if self.laser_system:
                laser_data = {
                    'point_cloud': self.laser_system.get_point_cloud(),
                    'nearest_distance': self.laser_system.get_nearest_obstacle_distance()
                }
            
            # Apply sensor fusion
            if laser_data:
                self._fuse_sensor_data(laser_data)
        else:
            logger.warning("Sensor synchronization timeout")
    
    def _fuse_sensor_data(self, laser_data):
        """Fuse camera and laser data for improved obstacle detection"""
        if not self.frame or not laser_data:
                return
        
        # Get camera-based obstacles
        camera_obstacles = self._detect_obstacles_from_camera(self.frame)
        
        # Get laser-based obstacles
        laser_obstacles = self._convert_laser_to_camera_coordinates(laser_data['point_cloud'])
        
        # Fuse obstacles using weighted combination
        fused_obstacles = []
        for cam_obs in camera_obstacles:
            # Find closest laser obstacle
            closest_laser = self._find_closest_obstacle(cam_obs, laser_obstacles)
            
            if closest_laser:
                # Weighted fusion
                fused_pos = (
                    self.config['fusion_weight_camera'] * cam_obs['position'][0] +
                    self.config['fusion_weight_laser'] * closest_laser[0],
                    self.config['fusion_weight_camera'] * cam_obs['position'][1] +
                    self.config['fusion_weight_laser'] * closest_laser[1]
                )
                
                # Fuse size information
                fused_size = (
                    max(cam_obs['size'][0], closest_laser[2]),
                    max(cam_obs['size'][1], closest_laser[3])
                )
                
                fused_obstacles.append({
                    'position': fused_pos,
                    'size': fused_size,
                    'confidence': max(cam_obs.get('confidence', 0.5), closest_laser[4])
                })
            else:
                # Keep camera obstacle if no laser match
                fused_obstacles.append(cam_obs)
        
        # Add unmatched laser obstacles
        for laser_obs in laser_obstacles:
            if not self._find_closest_obstacle(laser_obs, camera_obstacles):
                fused_obstacles.append({
                    'position': (laser_obs[0], laser_obs[1]),
                    'size': (laser_obs[2], laser_obs[3]),
                    'confidence': laser_obs[4]
                })
        
        self.obstacles = fused_obstacles
    
    def _convert_laser_to_camera_coordinates(self, point_cloud):
        """Convert laser point cloud to camera coordinates"""
        if not point_cloud or not self.camera_matrix:
            return []
        
        # Convert points to numpy array
        points = np.array(point_cloud)
        
        # Apply camera calibration
        if self.distortion_coeffs is not None:
            points = cv2.undistortPoints(points, self.camera_matrix, self.distortion_coeffs)
        
        # Convert to camera coordinates
        camera_points = []
        for point in points:
            # Apply coordinate transformation
            # This would need to be calibrated based on the physical setup
            x = point[0] * 0.1  # Scale factor
            y = point[1] * 0.1
            camera_points.append([x, y])
        
        return camera_points
    
    def _find_closest_obstacle(self, obstacle, obstacle_list):
        """Find the closest obstacle from a list"""
        if not obstacle_list:
            return None
        
        min_dist = float('inf')
        closest = None
        
        for obs in obstacle_list:
            dist = np.sqrt(
                (obstacle['position'][0] - obs[0])**2 +
                (obstacle['position'][1] - obs[1])**2
            )
            if dist < min_dist:
                min_dist = dist
                closest = obs
        
        return closest if min_dist < 1.0 else None  # 1 meter threshold
    
    def calibrate(self):
        """Calibrate the camera system"""
        logger.info("Starting camera calibration")
        
        # Create calibration directory if it doesn't exist
        os.makedirs("calibration", exist_ok=True)
        
        # Initialize calibration data
        calibration_images = []
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.001)
        
        # Prepare object points
        objp = np.zeros((self.config['calibration_grid_size'][0] * 
                        self.config['calibration_grid_size'][1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.config['calibration_grid_size'][0],
                              0:self.config['calibration_grid_size'][1]].T.reshape(-1, 2)
        objp *= self.config['calibration_square_size']
        
        # Capture calibration images
        image_count = 0
        while image_count < self.config['max_calibration_images']:
            try:
                # Capture frame
                frame = self._capture_frame()
                if frame is None:
                    continue
                
                # Convert to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Find chessboard corners
                ret, corners = cv2.findChessboardCorners(
                    gray,
                    self.config['calibration_grid_size'],
                    None
                )
                
                if ret:
                    # Refine corner detection
                    corners2 = cv2.cornerSubPix(
                        gray,
                        corners,
                        (11, 11),
                        (-1, -1),
                        criteria
                    )
                    
                    # Check corner quality
                    if self._check_corner_quality(corners2):
                        calibration_images.append((objp, corners2))
                        image_count += 1
                        logger.info(f"Calibration image {image_count} captured")
                        
                        # Save calibration image
                        cv2.imwrite(f"calibration/calibration_{image_count}.jpg", frame)
                
                time.sleep(1)  # Wait before next capture
            
            except Exception as e:
                logger.error(f"Error during calibration capture: {e}")
                continue
        
        if len(calibration_images) < self.config['min_calibration_images']:
            logger.error("Not enough calibration images captured")
            return False
        
        # Perform camera calibration
        try:
            objpoints = [img[0] for img in calibration_images]
            imgpoints = [img[1] for img in calibration_images]
            
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints,
                imgpoints,
                gray.shape[::-1],
                None,
                None
            )
            
            # Save calibration data
            calibration_data = {
                'camera_matrix': mtx.tolist(),
                'distortion_coeffs': dist.tolist(),
                'rvecs': [r.tolist() for r in rvecs],
                'tvecs': [t.tolist() for t in tvecs]
            }
            
            with open('calibration/camera_calibration.json', 'w') as f:
                json.dump(calibration_data, f)
            
            # Update internal calibration data
            self.camera_matrix = mtx
            self.distortion_coeffs = dist
            self.calibration_data = calibration_data
            
            logger.info("Camera calibration completed successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error during camera calibration: {e}")
            return False
    
    def _check_corner_quality(self, corners):
        """Check the quality of detected corners"""
        if corners is None:
            return False
        
        # Calculate corner quality metrics
        corner_quality = cv2.cornerMinEigenVal(corners)
        avg_quality = np.mean(corner_quality)
        
        return avg_quality > self.config['calibration_quality_threshold']
    
    def _detect_obstacles_from_camera(self, frame):
        """Detect obstacles using camera data"""
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Apply Canny edge detection
            edges = cv2.Canny(blurred, 50, 150)
            
            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            obstacles = []
            
            # Process contours to find obstacles
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Only consider if area is large enough
                if area > self.config['min_obstacle_area']:
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate obstacle position and size
                    position = (x + w/2, y + h/2)
                    size = (w, h)
                    
                    # Estimate distance using size
                    distance = self._estimate_distance(size)
                    
                    # Only include if within valid range
                    if self.config['min_obstacle_distance'] <= distance <= self.config['max_obstacle_distance']:
                        obstacles.append({
                            'position': position,
                            'size': size,
                            'distance': distance,
                            'confidence': self._calculate_confidence(area, distance)
                        })
            
            return obstacles
            
        except Exception as e:
            logger.error(f"Error in obstacle detection: {e}")
            return []
    
    def _estimate_distance(self, size):
        """Estimate distance to obstacle based on its apparent size"""
        # This is a simplified estimation - in a real system, this would use
        # proper depth estimation or stereo vision
        known_size = 0.5  # meters (typical obstacle size)
        focal_length = self.camera_matrix[0, 0] if self.camera_matrix is not None else 1000
        return (known_size * focal_length) / size[0]
    
    def _calculate_confidence(self, area, distance):
        """Calculate confidence in obstacle detection"""
        # Confidence based on area and distance
        area_confidence = min(1.0, area / (self.config['min_obstacle_area'] * 2))
        distance_confidence = 1.0 - (distance / self.config['max_obstacle_distance'])
        
        return (area_confidence + distance_confidence) / 2
    
    def get_performance_metrics(self):
        """Get system performance metrics"""
        return {
            'frame_count': self.frame_count,
            'error_count': self.error_count,
            'error_rate': self.error_count / max(1, self.frame_count),
            'avg_processing_time': np.mean(self.processing_times) if self.processing_times else 0,
            'last_sync_time': self.last_sync_time,
            'calibration_status': 'calibrated' if self.calibration_data else 'uncalibrated'
        }
