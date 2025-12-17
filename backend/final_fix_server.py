import uvicorn
from src.api.main import app

if __name__ == "__main__":
    print("Starting server on 127.0.0.1:8000")
    print("Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)")
    
    # Explicitly set host and port to ensure it doesn't bind to 0.0.0.0
    uvicorn.run(
        app="src.api.main:app",
        host="127.0.0.1",
        port=8000,
        reload=True,
        log_level="info"
    )