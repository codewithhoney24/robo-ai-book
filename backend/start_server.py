import sys
import os
sys.path.insert(0, os.getcwd())

# Load environment
from src.config.settings import settings

# Import and run the app with uvicorn
from src.api.main import app
import uvicorn

# Ensure we always bind to 127.0.0.1 (localhost) for local development
host = settings.host if settings.host != "0.0.0.0" else "127.0.0.1"
port = settings.port

print(f'Starting server on {host}:{port}')
print(f'Server running at: http://{host}:{port}')
print('Press Ctrl+C to stop the server')

if __name__ == "__main__":
    uvicorn.run(
        app,
        host=host,
        port=port,
        log_level='info',
        reload=False
    )