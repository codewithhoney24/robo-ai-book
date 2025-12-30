@echo off
REM Railway Deployment Script for Robo-AI-Book Platform

echo Starting deployment of Robo-AI-Book Platform to Railway...

REM Check if logged in to Railway
railway whoami
if %errorlevel% neq 0 (
    echo You are not logged in to Railway. Please run 'railway login' first.
    exit /b 1
)

echo Logged in to Railway successfully.

REM Check if in the correct directory
if not exist "railway.json" (
    echo railway.json not found. Please run this script from the project root.
    exit /b 1
)

echo Found railway.json configuration.

REM Deploy the application
echo Deploying application to Railway...
railway up

if %errorlevel% equ 0 (
    echo Deployment successful!
    echo Your application is now deployed to Railway.
    for /f "tokens=*" %%i in ('railway domain') do set DOMAIN=%%i
    echo You can view your deployment at: %DOMAIN%
) else (
    echo Deployment failed. Please check the logs with 'railway logs'.
    exit /b 1
)

echo Deployment process completed.