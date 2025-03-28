#!/usr/bin/env python3
"""
Run Ground Station
Script to run the ground station interface
"""
import logging
from ground_station import create_app

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("RunGroundStation")

if __name__ == "__main__":
    logger.info("Starting ground station...")
    
    # Create app
    app = create_app()
    
    # Run app
    logger.info("Ground station running at http://localhost:8080")
    app.run(host="0.0.0.0", port=8080, debug=True)

