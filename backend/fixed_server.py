#!/usr/bin/env python3
"""
Fixed server startup script that explicitly binds to 127.0.0.1:8000
"""
import uvicorn

def main():
    """
    Run the FastAPI application using uvicorn with explicit localhost binding.
    """
    print("Starting server on 127.0.0.1:8000")
    print("Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)")

    # Use import string instead of app object to enable reload
    uvicorn.run(
        "src.api.main:app",
        host="127.0.0.1",  # Explicitly set to localhost
        port=8000,         # Explicitly set port
        reload=True,       # Enable auto-reload for development
        workers=1,
        log_level="info"
    )

if __name__ == "__main__":
    main()