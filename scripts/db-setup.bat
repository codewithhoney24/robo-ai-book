@echo off
echo Setting up database migrations with Drizzle Kit...

REM Ensure we're in the correct directory
cd /d "%~dp0"

REM Check if Node.js and npm are available
node --version >nul 2>&1
if errorlevel 1 (
    echo Node.js is not installed or not in PATH
    pause
    exit /b 1
)

npm --version >nul 2>&1
if errorlevel 1 (
    echo npm is not installed or not in PATH
    pause
    exit /b 1
)

REM Install dependencies if node_modules doesn't exist
if not exist "node_modules" (
    echo Installing dependencies...
    npm install
)

REM Generate migrations
echo Generating database migrations...
npx drizzle-kit generate

REM Push schema to database (for development)
echo Pushing schema to database (development only)...
npx drizzle-kit push

echo Database setup complete!
pause