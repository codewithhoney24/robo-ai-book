from src.config.settings import settings

print("Current settings values:")
print(f"Host: {settings.host}")
print(f"Port: {settings.port}")
print(f"Debug: {settings.debug}")

# Let's also check the raw environment variables
import os
print("\nRaw environment variables:")
print(f"HOST: {os.getenv('HOST', 'not set')}")
print(f"PORT: {os.getenv('PORT', 'not set')}")
print(f"DEBUG: {os.getenv('DEBUG', 'not set')}")