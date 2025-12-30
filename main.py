#!/usr/bin/env python3
# This is the main entry point for the Python application
# This file helps Railway detect the project as Python-based

import os
import sys
from pathlib import Path

# Add the project root to the Python path to ensure modules can be found
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def main():
    """Main entry point for the application"""
    # Change to the backend directory
    backend_path = Path(__file__).parent / "backend"
    os.chdir(backend_path)

    # Import and run the server
    import uvicorn
    from src.api.main import app
    from src.config.settings import settings

    # Use the host and port from settings, defaulting to Railway's PORT environment variable
    host = settings.host if settings.host != "0.0.0.0" else "0.0.0.0"
    port = int(os.environ.get("PORT", settings.port))

    print(f"Starting server on {host}:{port}")
    print(f"Server running at: http://{host}:{port}")
    print("Press Ctrl+C to stop the server")

    uvicorn.run(
        app,
        host=host,
        port=port,
        log_level='info',
        reload=False
    )

if __name__ == "__main__":
    main()