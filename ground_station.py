#!/usr/bin/env python3
"""
Ground Station Interface
Web-based interface for monitoring and controlling the drone
"""
import os
import time
import json
import logging
import threading
import socket
import datetime
from flask import Flask, render_template, request, jsonify, Response

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("GroundStation")

class DroneClient:
    """Client for communicating with the drone"""
    
    def __init__(self, host="localhost", port=5000):
        """Initialize the drone client"""
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.telemetry_data = {}
        self.telemetry_history = {
            'altitude': [],
            'battery': [],
            'timestamps': []
        }
        self.lock = threading.Lock()
        self.max_history = 100  # Maximum number of history points to keep
    
    def connect(self):
        """Connect to the drone"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            
            # Start receiver thread
            self.receiver_thread = threading.Thread(target=self._receiver_loop)
            self.receiver_thread.daemon = True
            self.receiver_thread.start()
            
            logger.info(f"Connected to drone at {self.host}:{self.port}")
            return True
        
        except Exception as e:
            logger.error(f"Failed to connect to drone: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the drone"""
        if not self.connected:
            return
        
        self.connected = False
        
        # Close socket
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        logger.info("Disconnected from drone")
    
    def _receiver_loop(self):
        """Main receiver loop"""
        while self.connected:
            try:
                # Receive data
                data = self.socket.recv(4096)
                
                if not data:
                    # Connection closed
                    logger.warning("Connection closed by drone")
                    self.connected = False
                    break
                
                # Process received data
                self._process_data(data)
                
            except Exception as e:
                logger.error(f"Error receiving data: {e}")
                time.sleep(1)
    
    def _process_data(self, data):
        """Process data received from the drone"""
        try:
            # Decode and parse JSON data
            message = json.loads(data.decode())
            
            # Check message type
            if message.get("type") == "telemetry":
                # Update telemetry data
                with self.lock:
                    self.telemetry_data = message.get("data", {})
                    
                    # Add to history
                    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
                    
                    if 'altitude' in self.telemetry_data:
                        self.telemetry_history['altitude'].append(self.telemetry_data['altitude'])
                    if 'battery' in self.telemetry_data:
                        self.telemetry_history['battery'].append(self.telemetry_data['battery'])
                    self.telemetry_history['timestamps'].append(timestamp)
                    
                    # Trim history if too long
                    if len(self.telemetry_history['timestamps']) > self.max_history:
                        self.telemetry_history['altitude'] = self.telemetry_history['altitude'][-self.max_history:]
                        self.telemetry_history['battery'] = self.telemetry_history['battery'][-self.max_history:]
                        self.telemetry_history['timestamps'] = self.telemetry_history['timestamps'][-self.max_history:]
        
        except json.JSONDecodeError:
            logger.error("Invalid JSON data received")
        
        except Exception as e:
            logger.error(f"Error processing data: {e}")
    
    def send_command(self, command):
        """Send a command to the drone"""
        if not self.connected:
            logger.error("Not connected to drone")
            return False
        
        try:
            # Prepare command message
            message = {
                "type": "command",
                "command": command
            }
            
            # Encode and send
            data = json.dumps(message).encode()
            self.socket.send(data)
            
            logger.info(f"Sent command: {command}")
            return True
            
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return False
    
    def get_telemetry(self):
        """Get the latest telemetry data"""
        with self.lock:
            return self.telemetry_data.copy()
    
    def get_telemetry_history(self):
        """Get the telemetry history"""
        with self.lock:
            return self.telemetry_history.copy()
    
    def is_connected(self):
        """Check if connected to the drone"""
        return self.connected


# Create Flask app
app = Flask(__name__)

# Create drone client
drone_client = DroneClient()

@app.route('/')
def index():
    """Render the main page"""
    return render_template('index.html')

@app.route('/api/connect', methods=['POST'])
def connect():
    """Connect to the drone"""
    data = request.json
    host = data.get('host', 'localhost')
    port = int(data.get('port', 5000))
    
    # Update client settings
    drone_client.host = host
    drone_client.port =  5000
    
    # Update client settings
    drone_client.host = host
    drone_client.port = port
    
    # Connect to drone
    success = drone_client.connect()
    
    if success:
        return jsonify({"status": "connected"})
    else:
        return jsonify({"status": "failed"}), 500

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """Disconnect from the drone"""
    drone_client.disconnect()
    return jsonify({"status": "disconnected"})

@app.route('/api/telemetry')
def telemetry():
    """Get the latest telemetry data"""
    data = drone_client.get_telemetry()
    return jsonify(data)

@app.route('/api/telemetry/history')
def telemetry_history():
    """Get the telemetry history"""
    data = drone_client.get_telemetry_history()
    return jsonify(data)

@app.route('/api/command', methods=['POST'])
def command():
    """Send a command to the drone"""
    if not drone_client.is_connected():
        return jsonify({"status": "error", "message": "Not connected to drone"}), 400
    
    data = request.json
    command = data.get('command')
    
    if not command:
        return jsonify({"status": "error", "message": "No command specified"}), 400
    
    success = drone_client.send_command(command)
    
    if success:
        return jsonify({"status": "sent"})
    else:
        return jsonify({"status": "failed"}), 500

@app.route('/api/status')
def status():
    """Get the connection status"""
    return jsonify({"connected": drone_client.is_connected()})


def create_app():
    """Create and configure the Flask app"""
    # Ensure the template folder exists
    os.makedirs('templates', exist_ok=True)
    
    # Create index.html if it doesn't exist
    index_path = os.path.join('templates', 'index.html')
    if not os.path.exists(index_path):
        with open(index_path, 'w') as f:
            f.write('''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Drone Ground Station</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0-alpha1/dist/css/bootstrap.min.css" rel="stylesheet">
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body {
            padding-top: 20px;
            background-color: #f5f5f5;
        }
        .card {
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        .card-header {
            background-color: #343a40;
            color: white;
        }
        .status-indicator {
            width: 15px;
            height: 15px;
            border-radius: 50%;
            display: inline-block;
            margin-right: 5px;
        }
        .status-connected {
            background-color: #28a745;
        }
        .status-disconnected {
            background-color: #dc3545;
        }
        .telemetry-value {
            font-size: 24px;
            font-weight: bold;
        }
        .control-btn {
            margin: 5px;
        }
        #map-container {
            height: 300px;
            background-color: #e9ecef;
            border-radius: 5px;
            position: relative;
        }
        #drone-marker {
            position: absolute;
            width: 20px;
            height: 20px;
            background-color: #007bff;
            border-radius: 50%;
            transform: translate(-50%, -50%);
        }
        .emergency-btn {
            background-color: #dc3545;
            color: white;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1 class="text-center mb-4">Drone Ground Station</h1>
        
        <div class="row">
            <div class="col-md-4">
                <div class="card">
                    <div class="card-header">
                        Connection
                    </div>
                    <div class="card-body">
                        <div class="mb-3">
                            <span class="status-indicator" id="connection-status"></span>
                            <span id="connection-text">Disconnected</span>
                        </div>
                        <div class="mb-3">
                            <input type="text" class="form-control" id="host-input" placeholder="Host" value="localhost">
                        </div>
                        <div class="mb-3">
                            <input type="number" class="form-control" id="port-input" placeholder="Port" value="5000">
                        </div>
                        <button class="btn btn-primary" id="connect-btn">Connect</button>
                        <button class="btn btn-secondary" id="disconnect-btn">Disconnect</button>
                    </div>
                </div>
                
                <div class="card">
                    <div class="card-header">
                        Basic Controls
                    </div>
                    <div class="card-body">
                        <div class="d-grid gap-2">
                            <button class="btn btn-primary control-btn" data-command='{"type":"mode_change","mode":"HOVER"}'>Hover</button>
                            <button class="btn btn-primary control-btn" data-command='{"type":"set_height","height":1.5}'>Set Height (1.5m)</button>
                            <button class="btn btn-primary control-btn" data-command='{"type":"set_home"}'>Set Home Position</button>
                            <button class="btn btn-primary control-btn" data-command='{"type":"mode_change","mode":"LANDING"}'>Land</button>
                            <button class="btn btn-primary control-btn" data-command='{"type":"mode_change","mode":"RETURN_TO_BASE"}'>Return to Base</button>
                            <button class="btn btn-danger emergency-btn control-btn" data-command='{"type":"mode_change","mode":"EMERGENCY"}'>EMERGENCY STOP</button>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="col-md-8">
                <div class="card">
                    <div class="card-header">
                        Telemetry
                    </div>
                    <div class="card-body">
                        <div class="row">
                            <div class="col-md-4 text-center">
                                <div>Altitude</div>
                                <div class="telemetry-value" id="altitude-value">0.0 m</div>
                            </div>
                            <div class="col-md-4 text-center">
                                <div>Battery</div>
                                <div class="telemetry-value" id="battery-value">0%</div>
                            </div>
                            <div class="col-md-4 text-center">
                                <div>Mode</div>
                                <div class="telemetry-value" id="mode-value">UNKNOWN</div>
                            </div>
                        </div>
                    </div>
                </div>
                
                <div class="card">
                    <div class="card-header">
                        Position
                    </div>
                    <div class="card-body">
                        <div id="map-container">
                            <div id="drone-marker"></div>
                        </div>
                        <div class="row mt-2">
                            <div class="col-md-4 text-center">
                                <div>X Position</div>
                                <div class="telemetry-value" id="x-position-value">0.0 m</div>
                            </div>
                            <div class="col-md-4 text-center">
                                <div>Y Position</div>
                                <div class="telemetry-value" id="y-position-value">0.0 m</div>
                            </div>
                            <div class="col-md-4 text-center">
                                <div>Heading</div>
                                <div class="telemetry-value" id="heading-value">0°</div>
                            </div>
                        </div>
                    </div>
                </div>
                
                <div class="card">
                    <div class="card-header">
                        Telemetry History
                    </div>
                    <div class="card-body">
                        <canvas id="telemetry-chart"></canvas>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        // Connection status
        let connected = false;
        
        // Update connection status UI
        function updateConnectionStatus() {
            const statusIndicator = document.getElementById('connection-status');
            const statusText = document.getElementById('connection-text');
            
            if (connected) {
                statusIndicator.className = 'status-indicator status-connected';
                statusText.textContent = 'Connected';
            } else {
                statusIndicator.className = 'status-indicator status-disconnected';
                statusText.textContent = 'Disconnected';
            }
        }
        
        // Connect to drone
        document.getElementById('connect-btn').addEventListener('click', async () => {
            const host = document.getElementById('host-input').value;
            const port = document.getElementById('port-input').value;
            
            try {
                const response = await fetch('/api/connect', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({ host, port })
                });
                
                const data = await response.json();
                
                if (data.status === 'connected') {
                    connected = true;
                    updateConnectionStatus();
                    startTelemetryUpdates();
                }
            } catch (error) {
                console.error('Connection error:', error);
            }
        });
        
        // Disconnect from drone
        document.getElementById('disconnect-btn').addEventListener('click', async () => {
            try {
                await fetch('/api/disconnect', {
                    method: 'POST'
                });
                
                connected = false;
                updateConnectionStatus();
            } catch (error) {
                console.error('Disconnection error:', error);
            }
        });
        
        // Send command to drone
        document.querySelectorAll('.control-btn').forEach(button => {
            button.addEventListener('click', async () => {
                if (!connected) {
                    alert('Not connected to drone');
                    return;
                }
                
                const commandStr = button.getAttribute('data-command');
                const command = JSON.parse(commandStr);
                
                try {
                    await fetch('/api/command', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json'
                        },
                        body: JSON.stringify({ command })
                    });
                } catch (error) {
                    console.error('Command error:', error);
                }
            });
        });
        
        // Update telemetry data
        async function updateTelemetry() {
            if (!connected) return;
            
            try {
                const response = await fetch('/api/telemetry');
                const data = await response.json();
                
                // Update altitude
                const altitudeValue = document.getElementById('altitude-value');
                if (data.altitude !== undefined) {
                    altitudeValue.textContent = data.altitude.toFixed(2) + ' m';
                }
                
                // Update battery
                const batteryValue = document.getElementById('battery-value');
                if (data.battery !== undefined) {
                    batteryValue.textContent = data.battery.toFixed(0) + '%';
                    
                    // Change color based on battery level
                    if (data.battery < 20) {
                        batteryValue.style.color = '#dc3545'; // Red
                    } else if (data.battery < 50) {
                        batteryValue.style.color = '#ffc107'; // Yellow
                    } else {
                        batteryValue.style.color = '#28a745'; // Green
                    }
                }
                
                // Update mode
                const modeValue = document.getElementById('mode-value');
                if (data.mode !== undefined) {
                    modeValue.textContent = data.mode;
                }
                
                // Update position
                const xPositionValue = document.getElementById('x-position-value');
                const yPositionValue = document.getElementById('y-position-value');
                const headingValue = document.getElementById('heading-value');
                
                if (data.position !== undefined) {
                    xPositionValue.textContent = data.position[0].toFixed(2) + ' m';
                    yPositionValue.textContent = data.position[1].toFixed(2) + ' m';
                    
                    // Update drone marker position
                    const droneMarker = document.getElementById('drone-marker');
                    const mapContainer = document.getElementById('map-container');
                    
                    // Map drone position to container coordinates
                    // Assuming map is 10m x 10m centered at (0,0)
                    const mapWidth = mapContainer.clientWidth;
                    const mapHeight = mapContainer.clientHeight;
                    
                    const x = (data.position[0] + 5) / 10 * mapWidth;
                    const y = mapHeight - (data.position[1] + 5) / 10 * mapHeight;
                    
                    droneMarker.style.left = x + 'px';
                    droneMarker.style.top = y + 'px';
                }
                
                if (data.orientation !== undefined) {
                    headingValue.textContent = data.orientation[2].toFixed(0) + '°';
                }
                
            } catch (error) {
                console.error('Telemetry error:', error);
            }
        }
        
        // Update telemetry chart
        let telemetryChart = null;
        
        async function updateTelemetryChart() {
            if (!connected) return;
            
            try {
                const response = await fetch('/api/telemetry/history');
                const data = await response.json();
                
                if (!data.timestamps || data.timestamps.length === 0) return;
                
                if (!telemetryChart) {
                    // Create chart
                    const ctx = document.getElementById('telemetry-chart').getContext('2d');
                    telemetryChart = new Chart(ctx, {
                        type: 'line',
                        data: {
                            labels: data.timestamps,
                            datasets: [
                                {
                                    label: 'Altitude (m)',
                                    data: data.altitude,
                                    borderColor: '#007bff',
                                    backgroundColor: 'rgba(0, 123, 255, 0.1)',
                                    borderWidth: 2,
                                    fill: true,
                                    yAxisID: 'y'
                                },
                                {
                                    label: 'Battery (%)',
                                    data: data.battery,
                                    borderColor: '#28a745',
                                    backgroundColor: 'rgba(40, 167, 69, 0.1)',
                                    borderWidth: 2,
                                    fill: true,
                                    yAxisID: 'y1'
                                }
                            ]
                        },
                        options: {
                            responsive: true,
                            scales: {
                                x: {
                                    display: true,
                                    title: {
                                        display: true,
                                        text: 'Time'
                                    }
                                },
                                y: {
                                    display: true,
                                    position: 'left',
                                    title: {
                                        display: true,
                                        text: 'Altitude (m)'
                                    }
                                },
                                y1: {
                                    display: true,
                                    position: 'right',
                                    title: {
                                        display: true,
                                        text: 'Battery (%)'
                                    },
                                    min: 0,
                                    max: 100
                                }
                            }
                        }
                    });
                } else {
                    // Update chart
                    telemetryChart.data.labels = data.timestamps;
                    telemetryChart.data.datasets[0].data = data.altitude;
                    telemetryChart.data.datasets[1].data = data.battery;
                    telemetryChart.update();
                }
                
            } catch (error) {
                console.error('Telemetry chart error:', error);
            }
        }
        
        // Start telemetry updates
        function startTelemetryUpdates() {
            // Update telemetry every second
            setInterval(updateTelemetry, 1000);
            
            // Update telemetry chart every 5 seconds
            setInterval(updateTelemetryChart, 5000);
        }
        
        // Check initial connection status
        async function checkConnectionStatus() {
            try {
                const response = await fetch('/api/status');
                const data = await response.json();
                
                connected = data.connected;
                updateConnectionStatus();
                
                if (connected) {
                    startTelemetryUpdates();
                }
            } catch (error) {
                console.error('Status error:', error);
            }
        }
        
        // Initialize
        checkConnectionStatus();
    </script>
</body>
</html>
            ''')
    
    return app


if __name__ == "__main__":
    # Create app
    app = create_app()
    
    # Run app
    app.run(host="0.0.0.0", port=8080, debug=True)

