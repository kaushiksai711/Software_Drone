#!/usr/bin/env python3
"""
Test Suite
Comprehensive tests for the drone control system
"""
import time
import logging
import unittest
import threading
import numpy as np
from unittest.mock import MagicMock, patch

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("TestSuite")

# Import modules to test
from main import DroneController, DroneMode
from flight_controller import FlightController
from camera_system import CameraSystem
from laser_system import LaserSystem
from navigation import NavigationSystem
from emergency import EmergencySystem
from communication import CommunicationSystem
from path_planning import PathPlanner
from box_detection import BoxDetector
from obstacle_avoidance import ObstacleAvoidance

class TestFlightController(unittest.TestCase):
    """Tests for the Flight Controller module"""
    
    @patch('serial.Serial')
    def test_initialization(self, mock_serial):
        """Test flight controller initialization"""
        # Setup mock
        mock_serial.return_value.is_open = True
        
        # Create flight controller
        fc = FlightController(port="COM1")
        
        # Verify initialization
        self.assertIsNotNone(fc)
        self.assertEqual(fc.port, "COM1")
        self.assertFalse(fc.is_armed)
        
        # Verify serial connection
        mock_serial.assert_called_once()
    
    @patch('serial.Serial')
    def test_arm_disarm(self, mock_serial):
        """Test arming and disarming"""
        # Setup mock
        mock_serial.return_value.is_open = True
        
        # Create flight controller
        fc = FlightController(port="COM1")
        
        # Test arming
        fc.arm()
        self.assertTrue(fc.is_armed)
        
        # Test disarming
        fc.disarm()
        self.assertFalse(fc.is_armed)
    
    @patch('serial.Serial')
    def test_set_controls(self, mock_serial):
        """Test setting control values"""
        # Setup mock
        mock_serial.return_value.is_open = True
        
        # Create flight controller
        fc = FlightController(port="COM1")
        
        # Test setting controls
        fc.set_controls(500, 100, -100, 50)
        
        # Verify values are clamped correctly
        self.assertEqual(fc.throttle, 500)
        self.assertEqual(fc.pitch, 100)
        self.assertEqual(fc.roll, -100)
        self.assertEqual(fc.yaw, 50)
        
        # Test clamping
        fc.set_controls(1500, 600, -600, 600)
        self.assertEqual(fc.throttle, 1000)  # Clamped to max
        self.assertEqual(fc.pitch, 500)      # Clamped to max
        self.assertEqual(fc.roll, -500)      # Clamped to min
        self.assertEqual(fc.yaw, 500)        # Clamped to max
    
    @patch('serial.Serial')
    def test_emergency_stop(self, mock_serial):
        """Test emergency stop"""
        # Setup mock
        mock_serial.return_value.is_open = True
        mock_serial.return_value.write = MagicMock()
        
        # Create flight controller
        fc = FlightController(port="COM1")
        
        # Arm and set throttle
        fc.arm()
        fc.set_controls(500, 0, 0, 0)
        
        # Test emergency stop
        fc.emergency_stop()
        
        # Verify state
        self.assertFalse(fc.is_armed)
        self.assertEqual(fc.throttle, 0)
        
        # Verify emergency command was sent
        mock_serial.return_value.write.assert_called_with(b"STOP\n")


class TestCameraSystem(unittest.TestCase):
    """Tests for the Camera System module"""
    
    @patch('requests.get')
    def test_initialization(self, mock_get):
        """Test camera system initialization"""
        # Create camera system
        cs = CameraSystem(camera_url="http://test-camera")
        
        # Verify initialization
        self.assertIsNotNone(cs)
        self.assertEqual(cs.camera_url, "http://test-camera")
        self.assertFalse(cs.is_running)
    
    @patch('requests.get')
    @patch('cv2.cvtColor')
    @patch('numpy.array')
    @patch('PIL.Image.open')
    def test_capture_frame(self, mock_image_open, mock_np_array, mock_cv2_cvtcolor, mock_get):
        """Test frame capture"""
        # Setup mocks
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.content = b"test_image_data"
        mock_get.return_value = mock_response
        
        mock_image = MagicMock()
        mock_image_open.return_value = mock_image
        
        mock_np_array.return_value = np.zeros((480, 640, 3), dtype=np.uint8)
        
        mock_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        mock_cv2_cvtcolor.return_value = mock_frame
        
        # Create camera system
        cs = CameraSystem(camera_url="http://test-camera")
        
        # Call _capture_frame
        cs._capture_frame()
        
        # Verify frame was captured
        self.assertIsNotNone(cs.frame)
        mock_get.assert_called_once_with("http://test-camera", timeout=1)
    
    def test_box_detection(self):
        """Test box detection"""
        # Create test frame with a yellow box
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw yellow box (BGR format)
        frame[100:200, 200:300] = [0, 255, 255]  # Yellow in BGR
        
        # Create camera system
        cs = CameraSystem()
        
        # Mock frame
        cs.frame = frame
        
        # Process frame
        with patch('cv2.cvtColor', return_value=frame):
            with patch('cv2.inRange', return_value=np.ones((480, 640), dtype=np.uint8)):
                with patch('cv2.findContours') as mock_find_contours:
                    # Mock contours
                    contour = np.array([[[200, 100]], [[300, 100]], [[300, 200]], [[200, 200]]])
                    mock_find_contours.return_value = ([contour], None)
                    
                    # Call _detect_landing_box
                    cs._detect_landing_box(frame)
        
        # Verify box was detected
        self.assertTrue(cs.box_detected)
        self.assertIsNotNone(cs.box_position)


class TestLaserSystem(unittest.TestCase):
    """Tests for the Laser System module"""
    
    @patch('serial.Serial')
    def test_initialization(self, mock_serial):
        """Test laser system initialization"""
        # Setup mock
        mock_serial.return_value.is_open = True
        
        # Create laser system
        ls = LaserSystem(port="COM2")
        
        # Verify initialization
        self.assertIsNotNone(ls)
        self.assertEqual(ls.port, "COM2")
        self.assertFalse(ls.is_running)
        
        # Verify serial connection
        mock_serial.assert_called_once()
    
    @patch('serial.Serial')
    def test_read_laser_data(self, mock_serial):
        """Test reading laser data"""
        # Setup mock
        mock_serial.return_value.is_open = True
        mock_serial.return_value.in_waiting = 10
        mock_serial.return_value.readline.return_value = b"D:1.5,45\n"
        
        # Create laser system
        ls = LaserSystem(port="COM2")
        
        # Call _read_laser_data
        ls._read_laser_data()
        
        # Verify data was read
        self.assertEqual(len(ls.distances), 1)
        self.assertEqual(len(ls.angles), 1)
        self.assertEqual(ls.distances[0], 1.5)
        self.assertEqual(ls.angles[0], 45)
    
    @patch('serial.Serial')
    def test_process_laser_data(self, mock_serial):
        """Test processing laser data"""
        # Setup mock
        mock_serial.return_value.is_open = True
        
        # Create laser system
        ls = LaserSystem(port="COM2")
        
        # Set test data
        ls.distances = [1.0, 2.0, 3.0]
        ls.angles = [0, 90, 180]
        
        # Call _process_laser_data
        ls._process_laser_data()
        
        # Verify point cloud was created
        self.assertEqual(len(ls.point_cloud), 3)
        
        # Check first point (distance=1.0, angle=0)
        self.assertAlmostEqual(ls.point_cloud[0][0], 1.0)  # x = distance * cos(angle)
        self.assertAlmostEqual(ls.point_cloud[0][1], 0.0)  # y = distance * sin(angle)
        
        # Check second point (distance=2.0, angle=90)
        self.assertAlmostEqual(ls.point_cloud[1][0], 0.0, places=5)  # x = distance * cos(angle)
        self.assertAlmostEqual(ls.point_cloud[1][1], 2.0)  # y = distance * sin(angle)


class TestNavigationSystem(unittest.TestCase):
    """Tests for the Navigation System module"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Create mock dependencies
        self.mock_camera = MagicMock()
        self.mock_laser = MagicMock()
        
        # Create navigation system
        self.nav = NavigationSystem(self.mock_camera, self.mock_laser)
        
        # Set initial position
        self.nav.current_position = (0, 0, 1.5)
        self.nav.current_orientation = (0, 0, 0)
    
    def test_start_hovering(self):
        """Test starting hover mode"""
        # Call start_hovering
        self.nav.start_hovering(2.0)
        
        # Verify state
        self.assertEqual(self.nav.state.name, "HOVERING")
        self.assertEqual(self.nav.hover_height, 2.0)
        self.assertFalse(self.nav.is_landed)
    
    def test_start_landing(self):
        """Test starting landing mode"""
        # Call start_landing
        self.nav.start_landing()
        
        # Verify state
        self.assertEqual(self.nav.state.name, "LANDING")
        self.assertFalse(self.nav.is_landed)
    
    def test_return_to_base(self):
        """Test returning to base"""
        # Set home position
        home = (10, 10, 2.0)
        
        # Call return_to_base
        self.nav.return_to_base(home)
        
        # Verify state
        self.assertEqual(self.nav.state.name, "RETURNING")
        self.assertEqual(self.nav.home_position, home)
    
    def test_set_path(self):
        """Test setting a path"""
        # Create test path
        path = [(1, 1, 2), (2, 2, 2), (3, 3, 2)]
        
        # Call set_path
        self.nav.set_path(path)
        
        # Verify state
        self.assertEqual(self.nav.state.name, "TRAVERSING")
        self.assertEqual(self.nav.path, path)
        self.assertEqual(self.nav.path_index, 0)


class TestEmergencySystem(unittest.TestCase):
    """Tests for the Emergency System module"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Create mock drone controller
        self.mock_drone = MagicMock()
        
        # Create emergency system
        self.emergency = EmergencySystem(self.mock_drone)
    
    def test_low_battery(self):
        """Test low battery detection"""
        # Set battery level to low
        self.emergency.battery_level = 15  # Below low threshold (20%)
        
        # Check battery
        self.emergency._check_battery()
        
        # Verify return to base was initiated
        self.mock_drone.set_mode.assert_called_once()
        self.assertTrue(self.emergency.emergency_active)
    
    def test_critical_battery(self):
        """Test critical battery detection"""
        # Set battery level to critical
        self.emergency.battery_level = 5  # Below critical threshold (10%)
        
        # Check battery
        self.emergency._check_battery()
        
        # Verify emergency landing was activated
        self.emergency.activate_emergency_landing.assert_called_once()
    
    def test_connection_loss(self):
        """Test connection loss detection"""
        # Set connection status
        self.emergency.connection_status = False
        self.emergency.last_connection_time = time.time() - 10  # 10 seconds ago (exceeds timeout)
        
        # Check connection
        self.emergency._check_connection()
        
        # Verify emergency landing was activated
        self.emergency.activate_emergency_landing.assert_called_once()


class TestPathPlanner(unittest.TestCase):
    """Tests for the Path Planner module"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Create path planner
        self.planner = PathPlanner()
        
        # Create test grid
        dimensions = (10, 10, 5)  # 10m x 10m x 5m
        obstacles = [(5, 5, 2, 1)]  # One obstacle at (5,5,2) with radius 1m
        self.planner.create_grid(dimensions, obstacles)
    
    def test_grid_creation(self):
        """Test grid creation"""
        # Verify grid dimensions
        self.assertEqual(self.planner.grid_dimensions, (20, 20, 10))  # Grid cells (0.5m per cell)
        
        # Verify obstacle is in grid
        self.assertTrue(self.planner.grid[10, 10, 4])  # Obstacle at (5,5,2) -> grid (10,10,4)
    
    def test_path_planning(self):
        """Test path planning"""
        # Define start and goal
        start = (1, 1, 1)
        goal = (8, 8, 3)
        
        # Plan path
        path = self.planner.plan_path(start, goal)
        
        # Verify path exists
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 0)
        
        # Verify path starts and ends at correct positions
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
    
    def test_path_smoothing(self):
        """Test path smoothing"""
        # Create test path
        path = [(0, 0, 1), (1, 0, 1), (2, 0, 1), (3, 0, 1), (4, 0, 1)]
        
        # Smooth path
        smoothed = self.planner.smooth_path(path)
        
        # Verify smoothed path has same length
        self.assertEqual(len(smoothed), len(path))
        
        # Verify start and end points are unchanged
        self.assertEqual(smoothed[0], path[0])
        self.assertEqual(smoothed[-1], path[-1])


class TestObstacleAvoidance(unittest.TestCase):
    """Tests for the Obstacle Avoidance module"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Create obstacle avoidance
        self.avoidance = ObstacleAvoidance()
    
    def test_avoid_obstacles(self):
        """Test obstacle avoidance"""
        # Define current position, target, and obstacles
        current = (0, 0, 1)
        target = (10, 0, 1)
        obstacles = [(5, 0, 1, 1)]  # Obstacle directly in path
        
        # Calculate avoidance vector
        vector = self.avoidance.avoid_obstacles(current, target, obstacles)
        
        # Verify vector is not directly towards target (has y component)
        self.assertNotEqual(vector[1], 0)
    
    def test_path_clear(self):
        """Test path clear check"""
        # Define start, end, and obstacles
        start = (0, 0, 1)
        end = (10, 0, 1)
        
        # Test with no obstacles
        self.assertTrue(self.avoidance.is_path_clear(start, end, []))
        
        # Test with obstacle in path
        obstacles = [(5, 0, 1, 1)]  # Obstacle directly in path
        self.assertFalse(self.avoidance.is_path_clear(start, end, obstacles))
        
        # Test with obstacle near but not in path
        obstacles = [(5, 2, 1, 0.5)]  # Obstacle near but not in path
        self.assertTrue(self.avoidance.is_path_clear(start, end, obstacles))


class TestBoxDetector(unittest.TestCase):
    """Tests for the Box Detector module"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Create box detector
        self.detector = BoxDetector()
    
    def test_detect_box(self):
        """Test box detection"""
        # Create test frame with a yellow box
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw yellow box (BGR format)
        frame[100:200, 200:300] = [0, 255, 255]  # Yellow in BGR
        
        # Mock OpenCV functions
        with patch('cv2.cvtColor', return_value=frame):
            with patch('cv2.inRange', return_value=np.ones((480, 640), dtype=np.uint8)):
                with patch('cv2.morphologyEx', return_value=np.ones((480, 640), dtype=np.uint8)):
                    with patch('cv2.findContours') as mock_find_contours:
                        # Mock contours
                        contour = np.array([[[200, 100]], [[300, 100]], [[300, 200]], [[200, 200]]])
                        mock_find_contours.return_value = ([contour], None)
                        
                        # Mock contourArea
                        with patch('cv2.contourArea', return_value=10000):
                            # Mock boundingRect
                            with patch('cv2.boundingRect', return_value=(200, 100, 100, 100)):
                                # Detect box
                                box_info = self.detector.detect_box(frame)
        
        # Verify box was detected
        self.assertIsNotNone(box_info)
        self.assertEqual(box_info['center'], (250, 150))
        self.assertEqual(box_info['size'], (100, 100))


class TestDroneController(unittest.TestCase):
    """Tests for the Drone Controller module"""
    
    @patch('flight_controller.FlightController')
    @patch('camera_system.CameraSystem')
    @patch('laser_system.LaserSystem')
    @patch('navigation.NavigationSystem')
    @patch('emergency.EmergencySystem')
    @patch('communication.CommunicationSystem')
    def test_initialization(self, mock_comm, mock_emergency, mock_nav, mock_laser, mock_camera, mock_flight):
        """Test drone controller initialization"""
        # Create drone controller
        drone = DroneController()
        
        # Verify initialization
        self.assertIsNotNone(drone)
        self.assertEqual(drone.mode, DroneMode.INITIALIZATION)
        
        # Verify subsystems were initialized
        mock_flight.assert_called_once()
        mock_camera.assert_called_once()
        mock_laser.assert_called_once()
        mock_nav.assert_called_once()
        mock_emergency.assert_called_once()
        mock_comm.assert_called_once()
    
    @patch('flight_controller.FlightController')
    @patch('camera_system.CameraSystem')
    @patch('laser_system.LaserSystem')
    @patch('navigation.NavigationSystem')
    @patch('emergency.EmergencySystem')
    @patch('communication.CommunicationSystem')
    def test_set_mode(self, mock_comm, mock_emergency, mock_nav, mock_laser, mock_camera, mock_flight):
        """Test setting drone mode"""
        # Create drone controller
        drone = DroneController()
        
        # Test setting hover mode
        drone.set_mode(DroneMode.HOVER)
        self.assertEqual(drone.mode, DroneMode.HOVER)
        mock_nav.return_value.start_hovering.assert_called_once()
        
        # Test setting landing mode
        drone.set_mode(DroneMode.LANDING)
        self.assertEqual(drone.mode, DroneMode.LANDING)
        mock_nav.return_value.start_landing.assert_called_once()
        
        # Test setting emergency mode
        drone.set_mode(DroneMode.EMERGENCY)
        self.assertEqual(drone.mode, DroneMode.EMERGENCY)
        mock_emergency.return_value.activate_emergency_landing.assert_called_once()
    
    @patch('flight_controller.FlightController')
    @patch('camera_system.CameraSystem')
    @patch('laser_system.LaserSystem')
    @patch('navigation.NavigationSystem')
    @patch('emergency.EmergencySystem')
    @patch('communication.CommunicationSystem')
    def test_process_hover_mode(self, mock_comm, mock_emergency, mock_nav, mock_laser, mock_camera, mock_flight):
        """Test processing hover mode"""
        # Create drone controller
        drone = DroneController()
        
        # Set hover mode
        drone.mode = DroneMode.HOVER
        
        # Mock navigation.get_current_height
        mock_nav.return_value.get_current_height.return_value = 1.0
        
        # Set target height
        drone.target_height = 2.0
        
        # Process hover mode
        drone.process_hover_mode()
        
        # Verify height adjustment
        mock_nav.return_value.adjust_height.assert_called_once_with(2.0)


def run_tests():
    """Run all tests"""
    # Create test suite
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTest(unittest.makeSuite(TestFlightController))
    suite.addTest(unittest.makeSuite(TestCameraSystem))
    suite.addTest(unittest.makeSuite(TestLaserSystem))
    suite.addTest(unittest.makeSuite(TestNavigationSystem))
    suite.addTest(unittest.makeSuite(TestEmergencySystem))
    suite.addTest(unittest.makeSuite(TestPathPlanner))
    suite.addTest(unittest.makeSuite(TestObstacleAvoidance))
    suite.addTest(unittest.makeSuite(TestBoxDetector))
    suite.addTest(unittest.makeSuite(TestDroneController))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result


if __name__ == "__main__":
    # Run tests
    result = run_tests()
    
    # Print summary
    print(f"\nTest Summary:")
    print(f"  Ran {result.testsRun} tests")
    print(f"  Failures: {len(result.failures)}")
    print(f"  Errors: {len(result.errors)}")
    
    # Exit with appropriate code
    if result.wasSuccessful():
        print("All tests passed!")
        exit(0)
    else:
        print("Tests failed!")
        exit(1)

