#!/usr/bin/env python3
"""
Emergency System
Handles emergency situations like low battery or connection loss
"""
import time
import logging
import threading

logger = logging.getLogger("Emergency")

class EmergencySystem:
    """Emergency system for the drone"""
    
    def __init__(self, drone_controller):
        """Initialize the emergency system"""
        self.drone_controller = drone_controller
        self.is_running = False
        self.battery_level = 100  # percentage
        self.connection_status = True  # True if connected
        self.last_connection_time = time.time()
        self.emergency_active = False
        
        # Thresholds
        self.low_battery_threshold = 20  # percentage
        self.critical_battery_threshold = 10  # percentage
        self.connection_timeout = 5.0  # seconds
    
    def start(self):
        """Start the emergency system"""
        if self.is_running:
            return
        
        self.is_running = True
        self.emergency_thread = threading.Thread(target=self._emergency_loop)
        self.emergency_thread.daemon = True
        self.emergency_thread.start()
        
        logger.info("Emergency system started")
    
    def stop(self):
        """Stop the emergency system"""
        if not self.is_running:
            return
        
        self.is_running = False
        
        # Wait for emergency thread to finish
        if hasattr(self, 'emergency_thread') and self.emergency_thread.is_alive():
            self.emergency_thread.join(timeout=2.0)
        
        logger.info("Emergency system stopped")
    
    def _emergency_loop(self):
        """Main emergency monitoring loop"""
        while self.is_running:
            try:
                # Check battery level
                self._check_battery()
                
                # Check connection status
                self._check_connection()
                
                # Sleep to maintain monitoring frequency
                time.sleep(1.0)  # 1Hz monitoring rate
                
            except Exception as e:
                logger.error(f"Error in emergency monitoring: {e}")
                time.sleep(1)  # Wait before retrying
    
    def _check_battery(self):
        """Check battery level and take action if needed"""
        # In a real system, this would read from a battery sensor
        # For this example, we'll simulate battery drain
        self.battery_level -= 0.01  # Simulate 1% drain per 100 seconds
        
        # Ensure battery level doesn't go below 0
        self.battery_level = max(0, self.battery_level)
        
        # Check if battery is critical
        if self.battery_level <= self.critical_battery_threshold:
            logger.critical(f"CRITICAL BATTERY LEVEL: {self.battery_level:.1f}%")
            self.activate_emergency_landing()
        
        # Check if battery is low
        elif self.battery_level <= self.low_battery_threshold:
            logger.warning(f"LOW BATTERY LEVEL: {self.battery_level:.1f}%")
            # If not already returning or landing, initiate return to base
            if not self.emergency_active:
                logger.warning("Low battery, initiating return to base")
                self.drone_controller.set_mode(self.drone_controller.DroneMode.RETURN_TO_BASE)
                self.emergency_active = True
    
    def _check_connection(self):
        """Check connection status and take action if needed"""
        # In a real system, this would check for heartbeat messages
        # For this example, we'll simulate connection status
        
        # Check if connection is lost
        if not self.connection_status:
            # Calculate time since last connection
            time_since_connection = time.time() - self.last_connection_time
            
            # Check if connection timeout exceeded
            if time_since_connection > self.connection_timeout:
                logger.critical(f"CONNECTION LOST FOR {time_since_connection:.1f}s")
                self.activate_emergency_landing()
    
    def update_connection_status(self, connected):
        """Update the connection status"""
        if connected:
            self.connection_status = True
            self.last_connection_time = time.time()
        else:
            self.connection_status = False
    
    def update_battery_level(self, level):
        """Update the battery level"""
        self.battery_level = level
    
    def should_activate_emergency(self):
        """Check if emergency mode should be activated"""
        # Check battery level
        if self.battery_level <= self.critical_battery_threshold:
            return True
        
        # Check connection status
        if not self.connection_status:
            time_since_connection = time.time() - self.last_connection_time
            if time_since_connection > self.connection_timeout:
                return True
        
        return False
    
    def activate_emergency_landing(self):
        """Activate emergency landing procedure"""
        if not self.emergency_active:
            logger.critical("ACTIVATING EMERGENCY LANDING")
            self.emergency_active = True
            
            # Initiate landing
            self.drone_controller.land_at_current_position()

