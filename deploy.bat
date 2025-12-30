@echo off
REM Deployment script for Railway on Windows

REM Install dependencies
pip install -r requirements.txt

REM Run database migrations if needed
cd backend
python create_tables.py

REM Start the application
python -m uvicorn src.api.main:app --host 0.0.0.0 --port %PORT%