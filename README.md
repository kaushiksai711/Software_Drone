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