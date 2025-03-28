#!/usr/bin/env python3
"""
Communication System
Handles communication with the ground station
"""
import time
import logging
import threading
import socket
import json

logger = logging.getLogger("Communication")

class CommunicationSystem:
    """Communication system for the drone"""
    
    def __init__(self, host="0.0.0.0", port=5000):
        """Initialize the communication system"""
        self.host = host
        self.port = port
        self.is_running = False
        self.socket = None
        self.clients = []
        self.command_queue = []
        self.lock = threading.Lock()
    
    def start(self):
        """Start the communication system"""
        if self.is_running:
            return
        
        self.is_running = True
        
        # Start server in a separate thread
        self.server_thread = threading.Thread(target=self._server_loop)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        logger.info("Communication system started")
    
    def stop(self):
        """Stop the communication system"""
        if not self.is_running:
            return
        
        self.is_running = False
        
        # Close all client connections
        with self.lock:
            for client in self.clients:
                try:
                    client.close()
                except:
                    pass
            self.clients = []
        
        # Close server socket
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        # Wait for server thread to finish
        if hasattr(self, 'server_thread') and self.server_thread.is_alive():
            self.server_thread.join(timeout=2.0)
        
        logger.info("Communication system stopped")
    
    def _server_loop(self):
        """Main server loop"""
        try:
            # Create socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(5)
            self.socket.settimeout(1.0)  # 1 second timeout for accept
            
            logger.info(f"Server listening on {self.host}:{self.port}")
            
            while self.is_running:
                try:
                    # Accept new connections
                    client, addr = self.socket.accept()
                    logger.info(f"New connection from {addr}")
                    
                    # Add client to list
                    with self.lock:
                        self.clients.append(client)
                    
                    # Start client handler thread
                    client_thread = threading.Thread(target=self._client_handler, args=(client, addr))
                    client_thread.daemon = True
                    client_thread.start()
                    
                except socket.timeout:
                    # Timeout on accept, just continue
                    pass
                except Exception as e:
                    logger.error(f"Error accepting connection: {e}")
                    time.sleep(1)
        
        except Exception as e:
            logger.error(f"Error in server loop: {e}")
        finally:
            # Ensure socket is closed
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
    
    def _client_handler(self, client, addr):
        """Handle client connection"""
        try:
            # Set timeout for client socket
            client.settimeout(1.0)
            
            while self.is_running:
                try:
                    # Receive data from client
                    data = client.recv(1024)
                    
                    if not data:
                        # Client disconnected
                        logger.info(f"Client {addr} disconnected")
                        break
                    
                    # Process received data
                    self._process_client_data(data, client)
                    
                except socket.timeout:
                    # Timeout on receive, just continue
                    pass
                except Exception as e:
                    logger.error(f"Error receiving data from client {addr}: {e}")
                    break
        
        except Exception as e:
            logger.error(f"Error in client handler for {addr}: {e}")
        finally:
            # Remove client from list
            with self.lock:
                if client in self.clients:
                    self.clients.remove(client)
            
            # Close client socket
            try:
                client.close()
            except:
                pass
    
    def _process_client_data(self, data, client):
        """Process data received from client"""
        try:
            # Decode and parse JSON data
            message = json.loads(data.decode())
            
            # Check message type
            if message.get("type") == "command":
                # Add command to queue
                with self.lock:
                    self.command_queue.append(message.get("command"))
                
                # Send acknowledgment
                response = {"status": "ok", "message": "Command received"}
                client.send(json.dumps(response).encode())
            
            elif message.get("type") == "status_request":
                # Send status response
                # This would include drone status information
                status = {
                    "status": "ok",
                    "battery": 85,  # Example value
                    "position": [0, 0, 1.5],  # Example value
                    "mode": "hover"  # Example value
                }
                client.send(json.dumps(status).encode())
        
        except json.JSONDecodeError:
            logger.error("Invalid JSON data received")
            # Send error response
            response = {"status": "error", "message": "Invalid JSON data"}
            client.send(json.dumps(response).encode())
        
        except Exception as e:
            logger.error(f"Error processing client data: {e}")
    
    def send_telemetry(self, telemetry_data):
        """Send telemetry data to all connected clients"""
        if not self.clients:
            return
        
        try:
            # Prepare telemetry message
            message = {
                "type": "telemetry",
                "data": telemetry_data
            }
            
            # Encode message
            data = json.dumps(message).encode()
            
            # Send to all clients
            with self.lock:
                for client in self.clients[:]:  # Copy list to avoid modification during iteration
                    try:
                        client.send(data)
                    except Exception as e:
                        logger.error(f"Error sending telemetry to client: {e}")
                        # Remove failed client
                        self.clients.remove(client)
                        try:
                            client.close()
                        except:
                            pass
        
        except Exception as e:
            logger.error(f"Error sending telemetry: {e}")
    
    def get_latest_command(self):
        """Get the latest command from the queue"""
        with self.lock:
            if self.command_queue:
                return self.command_queue.pop(0)
            return None

