# Drone Control System

A comprehensive Python-based drone control system that provides autonomous navigation, obstacle avoidance, and emergency handling capabilities. This system is designed to work with a KK2.1.5 flight controller and includes various safety features and monitoring systems.

## Features

- **Autonomous Navigation**: Path planning and execution with obstacle avoidance
- **Visual Detection**: Box detection for landing targets using computer vision
- **Distance Sensing**: Laser-based distance measurement and obstacle detection
- **Emergency Systems**: Multiple layers of safety features including:
  - Battery monitoring
  - Connection loss detection
  - Geofencing
  - Wind detection
  - Sensor failure monitoring
  - Vibration detection
- **Ground Station Interface**: Web-based control and monitoring interface
- **Comprehensive Testing**: Extensive test suite for all system components

## System Architecture

The system is modular and consists of several key components:

### Core Components

1. **Flight Controller Interface** (`flight_controller.py`)
   - Manages communication with the KK2.1.5 flight controller
   - Handles motor control and basic flight commands
   - Provides calibration and emergency stop functionality

2. **Camera System** (`camera_system.py`)
   - Interfaces with ESP32 camera module
   - Implements box detection for landing targets
   - Provides obstacle detection using computer vision

3. **Laser System** (`laser_system.py`)
   - Manages laser distance measurement
   - Provides point cloud data for obstacle detection
   - Real-time distance monitoring

4. **Navigation System** (`navigation.py`)
   - Handles path planning and execution
   - Implements PID control for position and orientation
   - Manages different navigation states (hovering, landing, etc.)

5. **Emergency Systems**
   - Basic Emergency System (`emergency.py`)
     - Battery monitoring
     - Connection loss detection
   - Enhanced Emergency System (`enhanced_emergency.py`)
     - Geofencing
     - Wind detection
     - Sensor failure monitoring
     - Vibration detection

6. **Communication System** (`communication.py`)
   - Manages communication with ground station
   - Handles command processing and telemetry transmission

7. **Ground Station** (`ground_station.py`)
   - Web-based interface for drone control
   - Real-time telemetry display
   - Command interface

### Supporting Components

1. **Box Detection** (`box_detection.py`)
   - Implements landing target detection algorithms
   - Uses color segmentation for box detection

2. **Obstacle Avoidance** (`obstacle_avoidance.py`)
   - Implements obstacle avoidance algorithms
   - Provides path clearance checking

3. **Path Planning** (`path_planning.py`)
   - A* path planning algorithm
   - Path smoothing and optimization

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd drone_control_system
```

2. Install required dependencies:
```bash
pip install -r requirements.txt
```

## Usage

### Starting the Drone System

1. Run the main drone controller:
```bash
python main.py
```

2. Start the ground station interface:
```bash
python run_ground_station.py
```

### Running Tests

Execute the test suite:
```bash
python run_tests.py
```

## Configuration

The system can be configured through various parameters in the respective component files:

- Flight controller settings in `flight_controller.py`
- Camera parameters in `camera_system.py`
- Navigation parameters in `navigation.py`
- Emergency thresholds in `emergency.py` and `enhanced_emergency.py`

## Safety Features

The system implements multiple layers of safety features:

1. **Basic Safety**
   - Battery monitoring
   - Connection loss detection
   - Emergency landing capability

2. **Enhanced Safety**
   - Geofencing to keep drone within defined boundaries
   - Wind detection and response
   - Sensor failure monitoring
   - Vibration detection

3. **Obstacle Avoidance**
   - Real-time obstacle detection
   - Path replanning when obstacles are detected
   - Safe distance maintenance

## Logging

The system uses Python's logging module for comprehensive logging:
- Logs are written to `drone.log`
- Console output for important events
- Different log levels for different types of information

## Hardware Integration Guide

### Hardware Requirements

1. **Flight Controller:**
   - KK2.1.5 flight controller or compatible
   - Serial interface via USB or UART
   - Gyroscope, accelerometer, and barometer sensors

2. **Camera System:**
   - ESP32 camera module or compatible
   - MJPEG stream over HTTP
   - Resolution: 640x480 or higher recommended

3. **Laser Distance System:**
   - 360° scanning laser module or multiple single-point lasers
   - Serial interface (USB or UART)
   - Range: Minimum 5m recommended

4. **On-board Computer:**
   - Raspberry Pi 4 (2GB+ RAM) or compatible
   - USB ports for peripherals
   - WiFi for ground station communication

5. **Power System:**
   - LiPo battery (3S or 4S recommended)
   - Power distribution board
   - Battery voltage/current monitoring

### Hardware Integration Steps

#### 1. Flight Controller Integration

```python
# Example interface in flight_controller.py
self.serial_port = serial.Serial(port, baud_rate, timeout=1)
```

**Implementation Steps:**
1. Connect the KK2.1.5 controller via USB or UART to the onboard computer
2. Update the `port` parameter in `flight_controller.py` with the correct serial port
3. Implement the required serial communication protocol based on KK2.1.5 documentation
4. Verify the following methods function correctly:
   - `arm()` / `disarm()`
   - `set_controls(throttle, pitch, roll, yaw)`
   - `get_sensor_data()`
   - `emergency_stop()`

#### 2. Camera System Integration

```python
# Example interface in camera_system.py
self.camera_url = "http://camera-ip-address:port/stream"
```

**Implementation Steps:**
1. Configure the ESP32 camera for MJPEG streaming over HTTP
2. Update the `camera_url` parameter with the correct IP and port
3. Test camera connectivity using the `_capture_frame()` method
4. Verify box detection calibration with `_detect_landing_box()`
5. Adjust the HSV color thresholds in `box_detection.py` for your specific landing target

#### 3. Laser System Integration

```python
# Example interface in laser_system.py
self.serial_port = serial.Serial(port, baud_rate, timeout=1)
```

**Implementation Steps:**
1. Connect the laser scanner via USB or UART to the onboard computer
2. Update the `port` parameter with the correct serial port
3. Implement the data parsing logic in `_read_laser_data()` to match your specific laser format
4. Verify distance measurements with `get_nearest_obstacle_distance()`
5. Test point cloud generation with `_process_laser_data()`

#### 4. Power System Integration

```python
# Example in emergency.py
self.battery_level = self._read_battery_voltage()
```

**Implementation Steps:**
1. Connect the battery voltage sensor to an ADC pin or I2C interface
2. Implement the voltage reading in `_read_battery_voltage()`
3. Calibrate the voltage-to-percentage conversion
4. Set appropriate thresholds for low and critical battery warnings in `emergency.py`

### Software Configuration

Update the following configuration parameters:

```python
# In flight_controller.py
SERIAL_PORT = "/dev/ttyUSB0"  # Update with actual port
BAUD_RATE = 115200            # Match with flight controller

# In camera_system.py
CAMERA_URL = "http://192.168.1.100:8080/stream"  # Update with actual camera IP

# In laser_system.py
LASER_PORT = "/dev/ttyUSB1"   # Update with actual port
LASER_BAUD = 115200           # Match with laser scanner

# In emergency.py
BATTERY_LOW_THRESHOLD = 20    # Percentage for RTB warning
BATTERY_CRITICAL_THRESHOLD = 10  # Percentage for emergency landing

# In enhanced_emergency.py
GEOFENCE_RADIUS = 50          # Maximum flight radius in meters
MAX_WIND_SPEED = 10           # Maximum allowed wind speed in m/s
MAX_VIBRATION = 3.0           # Maximum allowed vibration level
```

### Calibration Procedures

Execute the following calibration procedures before flight:

1. **Flight Controller Calibration:**
   ```python
   python calibrate_flight_controller.py
   ```

2. **Camera Calibration:**
   ```python
   python calibrate_camera.py
   ```

3. **Laser Calibration:**
   ```python
   python calibrate_laser.py
   ```

4. **Sensor Fusion Calibration:**
   ```python
   python calibrate_sensors.py
   ```

### Testing Hardware Integration

Test each hardware component individually:

```python
# Test flight controller
python test_flight_controller.py

# Test camera system
python test_camera.py

# Test laser system
python test_laser.py

# Run full integration tests
python run_tests.py --hardware
```

### Physical Integration Notes

1. **Component Placement:**
   - **Flight Controller**: Mount at center of gravity
   - **Camera**: Forward-facing, unobstructed view
   - **Laser Scanner**: 360° unobstructed placement or multiple single-point lasers
   - **Onboard Computer**: Vibration-isolated mount
   - **Power System**: Balanced weight distribution

2. **Wiring Guidelines:**
   - Keep signal wires away from power lines
   - Use shielded cables for sensor connections
   - Secure all connections with thread locker or hot glue
   - Label all connections for easy identification

3. **Cooling Considerations:**
   - Ensure adequate airflow for onboard computer
   - Consider heat sinks for high-power components
   - Monitor temperature during extended operation

## Troubleshooting

### Serial Connection Issues
- Verify the correct port is specified
- Check USB permissions: `sudo chmod 666 /dev/ttyUSB0`
- Test connection with: `python -m serial.tools.miniterm /dev/ttyUSB0 115200`

### Camera Connection Issues
- Verify the camera is powered and connected to the network
- Test the stream URL in a web browser
- Check camera settings using the ESP32 configuration page

### Sensor Calibration Issues
- Recalibrate using the calibration scripts
- Verify sensor mounting orientation
- Check for electromagnetic interference

## Flight Safety Procedures

### Pre-Flight Checklist
1. **Battery Check**: Ensure full charge
2. **Hardware Check**: Verify all connections
3. **Sensor Check**: Run diagnostic mode
4. **Range Check**: Test controls at 10m distance
5. **Area Check**: Verify safe flight area

### Emergency Procedures
1. **Manual Override**: Switch to manual mode via ground station
2. **Emergency Land**: Press emergency land button on ground station
3. **Kill Switch**: Use physical kill switch if equipped

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

[Add your license information here]

## Acknowledgments

- KK2.1.5 flight controller
- ESP32 camera module
- OpenCV for computer vision
- Flask for ground station interface
