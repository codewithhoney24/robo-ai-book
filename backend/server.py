import uvicorn
from src.api.main import app
from src.config.settings import settings


def start_server():
    """
    Start the FastAPI server with proper configuration to avoid address binding issues.
    """
    # Ensure we always bind to 127.0.0.1 (localhost) for local development
    host = settings.host if settings.host != "0.0.0.0" else "127.0.0.1"
    port = settings.port

    print(f"Starting server on {host}:{port}")
    print(f"Server running at: http://{host}:{port}")
    print("Press Ctrl+C to stop the server")

    uvicorn.run(
        "src.api.main:app",  # Reference to the app instance in main.py
        host=host,  # Use 127.0.0.1 instead of 0.0.0.0
        port=port,  # Usually 8000
        reload=settings.debug,  # Enable auto-reload during development when debug mode is on
        workers=1,  # Number of worker processes
        log_level="info",
        timeout_keep_alive=30,
    )


if __name__ == "__main__":
    start_server()