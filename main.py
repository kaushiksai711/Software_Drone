#!/usr/bin/env python3
"""
Drone Navigation and Control System
Main controller script that initializes and manages all subsystems
"""
import time
import threading
import logging
from enum import Enum
import os

# Import subsystems
from flight_controller import FlightController
from camera_system import CameraSystem
from laser_system import LaserSystem
from navigation import NavigationSystem
from emergency import EmergencySystem
from communication import CommunicationSystem
from enhanced_emergency import EnhancedEmergencySystem

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("drone.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("DroneMain")

class DroneMode(Enum):
    """Drone operation modes"""
    INITIALIZATION = 0
    MANUAL = 1
    COMMUNICATION = 2
    AUTONOMOUS = 3
    HOVER = 4
    TRAVERSE = 5
    LANDING = 6
    RETURN_TO_BASE = 7
    EMERGENCY = 8
    STANDBY = 9

class DroneController:
    """Main drone controller class that manages all subsystems"""
    
    def __init__(self):
        """Initialize the drone controller and all subsystems"""
        logger.info("Initializing Drone Control System")
        
        # Current mode
        self.mode = DroneMode.INITIALIZATION
        
        # Create config directory if it doesn't exist
        config_dir = "config"
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)
            logger.info(f"Created configuration directory: {config_dir}")
        
        # Initialize subsystems
        try:
            # Use environment variables or defaults for ports
            flight_controller_port = os.environ.get('FLIGHT_CONTROLLER_PORT', '/dev/ttyUSB0')
            laser_system_port = os.environ.get('LASER_SYSTEM_PORT', '/dev/ttyUSB1')
            camera_url = os.environ.get('CAMERA_URL', 'http://esp32-cam.local:80/stream')
            
            self.flight_controller = FlightController(port=flight_controller_port)
            self.camera_system = CameraSystem(camera_url=camera_url)
            self.laser_system = LaserSystem(port=laser_system_port)
            self.navigation = NavigationSystem(self.camera_system, self.laser_system)
            self.emergency = EmergencySystem(self)
            self.enhanced_emergency = EnhancedEmergencySystem(self)
            self.communication = CommunicationSystem()
            
            # System status
            self.is_running = False
            self.target_height = 1.0  # meters
            self.home_position = None
            self.landing_spot = None
            self.box_detected = False
            
            logger.info("All subsystems initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize subsystems: {e}")
            raise
    
    def start(self):
        """Start the drone control system"""
        logger.info("Starting Drone Control System")
        self.is_running = True
        
        # Start all subsystems
        self.flight_controller.start()
        self.camera_system.start()
        self.laser_system.start()
        self.navigation.start()
        self.emergency.start()
        self.enhanced_emergency.start()  # Start enhanced emergency system
        self.communication.start()
        
        # Start the main control loop in a separate thread
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        # Set initial mode
        self.set_mode(DroneMode.MANUAL)
        
        logger.info("Drone Control System started")
    
    def stop(self):
        """Stop the drone control system"""
        logger.info("Stopping Drone Control System")
        self.is_running = False
        
        # First disarm motors for safety
        try:
            if self.flight_controller.is_armed:
                self.flight_controller.disarm()
        except Exception as e:
            logger.error(f"Error disarming motors during shutdown: {e}")
        
        # Stop all subsystems with error handling
        for subsystem_name, subsystem in [
            ("flight_controller", self.flight_controller),
            ("camera_system", self.camera_system),
            ("laser_system", self.laser_system),
            ("navigation", self.navigation),
            ("emergency", self.emergency),
            ("enhanced_emergency", self.enhanced_emergency),
            ("communication", self.communication)
        ]:
            try:
                subsystem.stop()
                logger.info(f"Stopped {subsystem_name}")
            except Exception as e:
                logger.error(f"Error stopping {subsystem_name}: {e}")
        
        # Wait for control thread to finish
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            self.control_thread.join(timeout=5.0)
        
        logger.info("Drone Control System stopped")
    
    def set_mode(self, mode):
        """Change the drone operation mode"""
        logger.info(f"Changing mode from {self.mode} to {mode}")
        self.mode = mode
        
        # Perform mode-specific initialization
        if mode == DroneMode.HOVER:
            self.navigation.start_hovering(self.target_height)
        elif mode == DroneMode.LANDING:
            self.navigation.start_landing()
        elif mode == DroneMode.RETURN_TO_BASE:
            if self.home_position:
                self.navigation.return_to_base(self.home_position)
            else:
                logger.warning("Home position not set, cannot return to base")
                self.set_mode(DroneMode.HOVER)
        elif mode == DroneMode.EMERGENCY:
            self.emergency.activate_emergency_landing()
    
    def control_loop(self):
        """Main control loop that runs continuously"""
        logger.info("Starting main control loop")
        
        last_telemetry_time = 0
        
        while self.is_running:
            try:
                # Check for emergency conditions
                if self.emergency.should_activate_emergency():
                    self.set_mode(DroneMode.EMERGENCY)
                
                # Process mode-specific logic
                if self.mode == DroneMode.HOVER:
                    self.process_hover_mode()
                elif self.mode == DroneMode.TRAVERSE:
                    self.process_traverse_mode()
                elif self.mode == DroneMode.LANDING:
                    self.process_landing_mode()
                elif self.mode == DroneMode.AUTONOMOUS:
                    self.process_autonomous_mode()
                
                # Check for box detection
                if self.camera_system.is_box_detected() and not self.box_detected:
                    self.box_detected = True
                    logger.info("Box detected, preparing for landing")
                    self.landing_spot = self.camera_system.get_box_position()
                
                # Process communication commands
                self.process_communication()
                
                # Send telemetry data every 100ms
                current_time = time.time()
                if current_time - last_telemetry_time > 0.1:
                    self.send_telemetry()
                    last_telemetry_time = current_time
                
                # Sleep to maintain control loop frequency
                time.sleep(0.05)  # 20Hz control loop
                
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                # If critical error, activate emergency mode
                self.set_mode(DroneMode.EMERGENCY)

    def send_telemetry(self):
        """Send telemetry data to ground station"""
        try:
            # Gather telemetry data
            telemetry = {
                'altitude': self.navigation.get_current_height(),
                'position': self.navigation.get_current_position(),
                'orientation': self.navigation.current_orientation,
                'battery': self.emergency.battery_level,
                'mode': self.mode.name,
                'is_armed': self.flight_controller.is_armed,
                'box_detected': self.box_detected,
                'timestamp': time.time()
            }
            
            # Send telemetry data
            self.communication.send_telemetry(telemetry)
        except Exception as e:
            logger.error(f"Error sending telemetry: {e}")
    
    def process_hover_mode(self):
        """Process hover mode logic"""
        # Get current height
        current_height = self.navigation.get_current_height()
        
        # Adjust if needed
        if abs(current_height - self.target_height) > 0.1:
            self.navigation.adjust_height(self.target_height)
    
    def process_traverse_mode(self):
        """Process traverse mode logic"""
        # Check if path is completed
        if self.navigation.is_path_completed():
            logger.info("Path traversal completed")
            self.set_mode(DroneMode.HOVER)
        else:
            # Continue following path with obstacle avoidance
            self.navigation.follow_path()
    
    def process_landing_mode(self):
        """Process landing mode logic"""
        # Check if landing is completed
        if self.navigation.is_landed():
            logger.info("Landing completed")
            self.set_mode(DroneMode.STANDBY)
    
    def process_autonomous_mode(self):
        """Process autonomous mode logic"""
        # In autonomous mode, the drone follows a predefined mission
        # which may include hovering, traversing, and landing
        mission_status = self.navigation.get_mission_status()
        
        if mission_status == "completed":
            logger.info("Autonomous mission completed")
            self.set_mode(DroneMode.HOVER)
    
    def process_communication(self):
        """Process incoming communication commands"""
        command = self.communication.get_latest_command()
        if command:
            logger.info(f"Received command: {command}")
            
            if command.get("type") == "mode_change":
                new_mode = DroneMode[command.get("mode")]
                self.set_mode(new_mode)
            elif command.get("type") == "set_height":
                self.target_height = command.get("height")
                if self.mode == DroneMode.HOVER:
                    self.navigation.adjust_height(self.target_height)
            elif command.get("type") == "set_home":
                self.home_position = command.get("position")
            elif command.get("type") == "set_landing_spot":
                self.landing_spot = command.get("position")
    
    def calibrate_sensors(self):
        """Calibrate all sensors"""
        logger.info("Starting sensor calibration")
        
        # Calibrate each subsystem
        self.flight_controller.calibrate()
        self.camera_system.calibrate()
        self.laser_system.calibrate()
        
        logger.info("Sensor calibration completed")
    
    def set_target_height(self, height):
        """Set the target hovering height"""
        logger.info(f"Setting target height to {height} meters")
        self.target_height = height
        
        if self.mode == DroneMode.HOVER:
            self.navigation.adjust_height(height)
    
    def set_home_position(self):
        """Set current position as home position"""
        self.home_position = self.navigation.get_current_position()
        logger.info(f"Home position set to {self.home_position}")
    
    def land_at_current_position(self):
        """Land at the current position"""
        logger.info("Initiating landing at current position")
        self.set_mode(DroneMode.LANDING)
    
    def land_at_box(self):
        """Land at detected box position"""
        if self.box_detected:
            logger.info("Initiating landing at detected box")
            self.navigation.set_landing_target(self.camera_system.get_box_position())
            self.set_mode(DroneMode.LANDING)
        else:
            logger.warning("No box detected, cannot land at box")

    def check_hardware(self):
        """Check if all required hardware is connected"""
        hardware_status = {
            "flight_controller": self.flight_controller.serial and self.flight_controller.serial.is_open,
            "camera": self.camera_system.check_connection(),  # Add this method to CameraSystem
            "laser": self.laser_system.serial and self.laser_system.serial.is_open
        }
        
        missing_hardware = [hw for hw, status in hardware_status.items() if not status]
        
        if missing_hardware:
            logger.warning(f"Missing hardware: {', '.join(missing_hardware)}")
            return False
        else:
            logger.info("All hardware connected")
            return True

if __name__ == "__main__":
    # Create and start the drone controller
    drone = DroneController()
    
    try:
        # Calibrate sensors
        drone.calibrate_sensors()
        
        # Set home position
        drone.set_home_position()
        
        # Start the system
        drone.start()
        
        # Set initial hover height
        drone.set_target_height(1.5)  # 1.5 meters
        
        # Change to hover mode
        drone.set_mode(DroneMode.HOVER)
        
        # Keep the main thread running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, stopping drone")
    finally:
        # Ensure drone is stopped properly
        drone.stop()

