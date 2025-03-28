#!/usr/bin/env python3
"""
Flight Controller Interface
Handles communication with the KK2.1.5 flight controller
"""
import time
import logging
import serial
import threading
import serial.tools.list_ports
import platform
from unittest.mock import MagicMock

logger = logging.getLogger("FlightController")

class DummySerial:
    """Dummy serial connection for testing"""
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.in_waiting = 0
        self._buffer = []
    
    def write(self, data):
        """Simulate writing data"""
        logger.debug(f"Dummy write: {data}")
        return len(data)
    
    def readline(self):
        """Simulate reading data"""
        if self._buffer:
            return self._buffer.pop(0)
        return b"OK\n"
    
    def close(self):
        """Close the dummy connection"""
        self.is_open = False
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

class FlightController:
    """Interface to the KK2.1.5 flight controller"""
    
    def __init__(self, port=None, baudrate=115200, use_dummy=False):
        """Initialize the flight controller interface"""
        self.baudrate = baudrate
        self.serial = None
        self.is_running = False
        self.is_armed = False
        self.throttle = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.lock = threading.Lock()
        self.use_dummy = use_dummy
        
        # Set default port based on OS
        if port is None:
            if platform.system() == 'Windows':
                # Try to find the first available COM port
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    port = ports[0].device
                else:
                    port = 'COM1'  # Default to COM1 if no ports found
            else:
                port = '/dev/ttyUSB0'  # Linux default
        
        self.port = port
        
        # Connect to the flight controller
        self._connect()
    
    def _connect(self):
        """Connect to the flight controller via serial"""
        try:
            if self.use_dummy:
                logger.info("Using dummy serial connection")
                self.serial = DummySerial()
            else:
                # Try to connect to physical port
                try:
                    # Add retries for more robust connection
                    for attempt in range(3):
                        try:
                            self.serial = serial.Serial(
                                self.port,
                                self.baudrate,
                                timeout=1,
                                bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE
                            )
                            logger.info(f"Connected to flight controller on {self.port}")
                            break
                        except serial.SerialException as e:
                            if attempt < 2:  # Only retry if not the last attempt
                                logger.warning(f"Connection attempt {attempt+1} failed: {e}. Retrying...")
                                time.sleep(1)
                            else:
                                raise
                except serial.SerialException as e:
                    logger.warning(f"Failed to connect to physical port: {e}")
                    logger.info("Falling back to dummy connection")
                    self.serial = DummySerial()
            
            # Initialize dummy data if using dummy connection
            if isinstance(self.serial, DummySerial):
                self.serial._buffer = [
                    b"OK\n",
                    b"ARMED\n",
                    b"BATTERY: 85%\n",
                    b"ACC: 0.0, 0.0, 9.81\n",
                    b"GYRO: 0.0, 0.0, 0.0\n"
                ]
            
        except Exception as e:
            logger.error(f"Failed to connect to flight controller: {e}")
            # Try to list available ports for debugging
            try:
                ports = list(serial.tools.list_ports.comports())
                logger.info("Available ports:")
                for port in ports:
                    logger.info(f"  {port.device} - {port.description}")
            except Exception as port_error:
                logger.error(f"Error listing ports: {port_error}")
            raise
    
    def start(self):
        """Start the flight controller communication thread"""
        if self.is_running:
            return
        
        self.is_running = True
        self.comm_thread = threading.Thread(target=self._communication_loop)
        self.comm_thread.daemon = True
        self.comm_thread.start()
        
        logger.info("Flight controller communication started")
    
    def stop(self):
        """Stop the flight controller communication"""
        if not self.is_running:
            return
        
        # Disarm before stopping
        if self.is_armed:
            self.disarm()
        
        self.is_running = False
        
        # Wait for communication thread to finish
        if hasattr(self, 'comm_thread') and self.comm_thread.is_alive():
            self.comm_thread.join(timeout=2.0)
        
        # Close serial connection
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        logger.info("Flight controller communication stopped")
    
    def _communication_loop(self):
        """Main communication loop with the flight controller"""
        while self.is_running:
            try:
                with self.lock:
                    # Send control commands to the flight controller
                    self._send_control_commands()
                
                # Read and process any responses
                self._read_responses()
                
                # Sleep to maintain communication frequency
                time.sleep(0.02)  # 50Hz communication rate
                
            except Exception as e:
                logger.error(f"Error in flight controller communication: {e}")
                time.sleep(1)  # Wait before retrying
    
    def _send_control_commands(self):
        """Send control commands to the flight controller"""
        if not self.serial or not self.serial.is_open:
            return
        
        # Format control command for KK2.1.5
        # This is a simplified example - actual protocol may differ
        command = f"C,{self.throttle},{self.pitch},{self.roll},{self.yaw},{1 if self.is_armed else 0}\n"
        self.serial.write(command.encode())
    
    def _read_responses(self):
        """Read and process responses from the flight controller"""
        if not self.serial or not self.serial.is_open:
            return
        
        if self.serial.in_waiting > 0:
            try:
                response = self.serial.readline().decode().strip()
                if response:
                    self._process_response(response)
                    
                    # Update last reading time for sensor failure detection
                    if response.startswith("ACC:") or response.startswith("GYRO:"):
                        self.last_imu_reading_time = time.time()
                    elif response.startswith("BARO:"):
                        self.last_baro_reading_time = time.time()
            except Exception as e:
                logger.error(f"Error reading from flight controller: {e}")
    
    def _process_response(self, response):
        """Process a response from the flight controller"""
        # Example response processing - actual implementation will depend on KK2.1.5 protocol
        if response.startswith("S,"):
            # Status response
            parts = response.split(",")
            if len(parts) >= 5:
                # Update internal state based on controller feedback
                pass
    
    def arm(self):
        """Arm the flight controller"""
        with self.lock:
            logger.info("Arming flight controller")
            self.is_armed = True
            # Set minimum throttle
            self.throttle = 0
            # Send arm command
            self._send_control_commands()
            time.sleep(1)  # Wait for arming to complete
    
    def disarm(self):
        """Disarm the flight controller"""
        with self.lock:
            logger.info("Disarming flight controller")
            # Set minimum throttle
            self.throttle = 0
            # Send disarm command
            self.is_armed = False
            self._send_control_commands()
            time.sleep(1)  # Wait for disarming to complete
    
    def set_controls(self, throttle, pitch, roll, yaw):
        """Set control values for the flight controller"""
        with self.lock:
            # Clamp values to valid range (0-1000 for throttle, -500 to 500 for others)
            self.throttle = max(0, min(1000, throttle))
            self.pitch = max(-500, min(500, pitch))
            self.roll = max(-500, min(500, roll))
            self.yaw = max(-500, min(500, yaw))
    
    def calibrate(self):
        """Calibrate the flight controller"""
        logger.info("Calibrating flight controller")
        
        # Ensure disarmed
        self.disarm()
        
        # Send calibration command
        with self.lock:
            # Example calibration sequence - actual implementation will depend on KK2.1.5
            # Set all controls to neutral
            self.throttle = 0
            self.pitch = 0
            self.roll = 0
            self.yaw = 0
            
            # Send special calibration command if needed
            calibration_cmd = "CAL\n"
            if self.serial and self.serial.is_open:
                self.serial.write(calibration_cmd.encode())
            
            # Wait for calibration to complete
            time.sleep(5)
        
        logger.info("Flight controller calibration completed")
    
    def emergency_stop(self):
        """Emergency stop - immediately cut all motors"""
        with self.lock:
            logger.warning("Emergency stop activated")
            self.throttle = 0
            self.is_armed = False
            
            # Send emergency stop command
            emergency_cmd = "STOP\n"
            if self.serial and self.serial.is_open:
                self.serial.write(emergency_cmd.encode())

