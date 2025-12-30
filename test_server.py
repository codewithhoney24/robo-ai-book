#!/usr/bin/env python3
"""
Test script to verify the server can start properly
"""

import os
import sys
from pathlib import Path

def test_imports():
    """Test if all required modules can be imported"""
    try:
        # Add the project root to the Python path
        project_root = Path(__file__).parent
        backend_path = project_root / "backend"
        sys.path.insert(0, str(backend_path))

        print("Testing imports...")

        # Test importing the main app
        from src.api.main import app
        print("Successfully imported main app")

        # Test importing settings
        from src.config.settings import settings
        print("Successfully imported settings")

        # Test importing logging config
        from src.config.logging_config import api_logger
        print("Successfully imported logging config")

        # Test importing middleware
        from src.api.middleware.compatibility_check import compatibility_check_middleware
        print("Successfully imported middleware")

        print("\nAll imports successful!")
        return True

    except ImportError as e:
        print(f"Import error: {e}")
        import traceback
        traceback.print_exc()
        return False
    except Exception as e:
        print(f"Other error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_imports()
    if success:
        print("\nAll tests passed! The server should be able to start.")
    else:
        print("\nTests failed! There are issues that need to be fixed.")
        sys.exit(1)