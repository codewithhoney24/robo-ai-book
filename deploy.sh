#!/bin/bash
# Deployment script for Railway

# Install dependencies
pip install -r requirements.txt

# Run database migrations if needed
cd backend
python create_tables.py

# Start the application
exec uvicorn src.api.main:app --host 0.0.0.0 --port $PORT