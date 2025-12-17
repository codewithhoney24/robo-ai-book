@echo off
echo Starting the AI Book Backend Server...
echo.

REM Set environment variables
set DEBUG=True

REM Run the server with more detailed output
python -c "
import sys
import os
sys.path.insert(0, os.getcwd())

print('Python executable:', sys.executable)
print('Current working directory:', os.getcwd())
print('Python path:', sys.path[:3])  # Show first 3 paths

try:
    from src.config.settings import settings
    print('Settings loaded successfully')
    print('Host:', settings.host)
    print('Port:', settings.port)
except Exception as e:
    print('Error loading settings:', str(e))
    sys.exit(1)

try:
    import uvicorn
    print('Uvicorn imported successfully')
except Exception as e:
    print('Error importing uvicorn:', str(e))
    sys.exit(1)

try:
    from src.api.main import app
    print('FastAPI app imported successfully')
except Exception as e:
    print('Error importing app:', str(e))
    import traceback
    traceback.print_exc()
    sys.exit(1)

print('All imports successful, ready to start server...')
"