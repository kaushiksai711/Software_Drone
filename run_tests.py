#!/usr/bin/env python3
"""
Run Tests
Script to run the test suite
"""
import logging
import sys
from test_suite import run_tests

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("RunTests")

if __name__ == "__main__":
    logger.info("Running test suite...")
    
    # Run tests
    result = run_tests()
    
    # Exit with appropriate code
    if result.wasSuccessful():
        logger.info("All tests passed!")
        sys.exit(0)
    else:
        logger.error("Tests failed!")
        sys.exit(1)

