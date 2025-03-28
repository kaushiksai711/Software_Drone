#!/usr/bin/env python3
"""
Box Detection
Implements box detection algorithms for landing
"""
import cv2
import numpy as np
import logging

logger = logging.getLogger("BoxDetection")

class BoxDetector:
    """Box detector for landing target"""
    
    def __init__(self):
        """Initialize the box detector"""
        # HSV color range for yellow (landing box)
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        
        # Minimum box area (in pixels)
        self.min_box_area = 500
        
        # Aspect ratio range for box detection
        self.min_aspect_ratio = 0.7
        self.max_aspect_ratio = 1.3
    
    def detect_box(self, frame):
        """Detect landing box in the frame"""
        if frame is None:
            return None
        
        try:
            # Convert to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Create mask for yellow color
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            
            # Apply morphological operations to clean up the mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Process contours to find the landing box
            for contour in contours:
                # Calculate contour area
                area = cv2.contourArea(contour)
                
                # Skip if area is too small
                if area < self.min_box_area:
                    continue
                
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate aspect ratio
                aspect_ratio = float(w) / h
                
                # Check if aspect ratio is within range (roughly square)
                if self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio:
                    # Calculate center of the box
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Return box information
                    box_info = {
                        'center': (center_x, center_y),
                        'size': (w, h),
                        'area': area
                    }
                    
                    logger.info(f"Box detected at {box_info['center']} with size {box_info['size']}")
                    return box_info
            
            # No box found
            return None
            
        except Exception as e:
            logger.error(f"Error in box detection: {e}")
            return None
    
    def calculate_distance(self, box_info, camera_params):
        """Calculate distance to the box based on its apparent size"""
        if box_info is None:
            return None
        
        try:
            # Extract box size
            w, h = box_info['size']
            
            # Extract camera parameters
            focal_length = camera_params['focal_length']
            sensor_width = camera_params['sensor_width']
            
            # Known box size in meters
            real_box_size = 0.5  # 50cm x 50cm box
            
            # Calculate distance using similar triangles
            # distance = (real_box_size * focal_length) / (apparent_box_size_on_sensor)
            apparent_box_size_on_sensor = w * sensor_width / camera_params['image_width']
            distance = (real_box_size * focal_length) / apparent_box_size_on_sensor
            
            logger.info(f"Calculated distance to box: {distance:.2f}m")
            return distance
            
        except Exception as e:
            logger.error(f"Error calculating distance to box: {e}")
            return None

