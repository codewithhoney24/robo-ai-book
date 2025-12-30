# This is the main entry point for the Python application
# This file helps Railway detect the project as Python-based

import os
import sys
from pathlib import Path

# Add the backend directory to the Python path
backend_path = Path(__file__).parent / "backend"
sys.path.insert(0, str(backend_path))

def main():
    """Main entry point for the application"""
    os.chdir('backend')
    
    # Import and run the server
    from start_server import start_server
    start_server()

if __name__ == "__main__":
    main()