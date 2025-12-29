#!/usr/bin/env python3
"""
Test script to verify that the settings are loaded correctly.
"""
from src.config.settings import settings

def test_settings():
    print("Current settings:")
    print(f"SECRET_KEY: {settings.secret_key}")
    print(f"PORT: {settings.port}")

    # Check if secrets are properly loaded
    if not settings.secret_key or settings.secret_key == "":
        print("ERROR: SECRET_KEY is not set!")
    else:
        print(f"[OK] SECRET_KEY is set (length: {len(settings.secret_key)})")

if __name__ == "__main__":
    test_settings()