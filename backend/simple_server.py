import uvicorn
import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.api.main import app

if __name__ == "__main__":
    try:
        print("Starting server on http://127.0.0.1:8080")
        print("Press Ctrl+C to stop the server")
        uvicorn.run(
            app,
            host="127.0.0.1",
            port=8080,
            reload=True,
            log_level="info"
        )
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    except Exception as e:
        print(f"Error starting server: {e}")