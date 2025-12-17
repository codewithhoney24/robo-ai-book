import uvicorn
import os
from dotenv import load_dotenv # Ye line zaroori hai

# 1. Sab se pehle .env file load karein
load_dotenv()

# 2. Ab settings import karein (ab ye sahi values uthayega)
from src.api.main import app
from src.config.settings import settings

def start_server():
    # Double check karne ke liye hum print karwayenge ke environment mein kya hai
    env_host = os.getenv("HOST")
    print(f"DEBUG CHECK -> .env HOST val: {env_host}") 
    print(f"DEBUG CHECK -> settings HOST val: {settings.host}")

    print(f"Starting server on {settings.host}:{settings.port}")
    print(f"Server running at: http://{settings.host}:{settings.port}")
    print("Press Ctrl+C to stop the server")

    uvicorn.run(
        "src.api.main:app",
        host=settings.host,
        port=settings.port,
        reload=False,
        workers=1,
        log_level="debug",
        timeout_keep_alive=30,
    )

if __name__ == "__main__":
    start_server()