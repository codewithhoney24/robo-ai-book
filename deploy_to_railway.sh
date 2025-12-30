#!/bin/bash
# Railway Deployment Script for Robo-AI-Book Platform

echo "Starting deployment of Robo-AI-Book Platform to Railway..."

# Check if logged in to Railway
if ! railway whoami 2>/dev/null; then
    echo "You are not logged in to Railway. Please run 'railway login' first."
    exit 1
fi

echo "Logged in to Railway successfully."

# Check if in the correct directory
if [ ! -f "railway.json" ]; then
    echo "railway.json not found. Please run this script from the project root."
    exit 1
fi

echo "Found railway.json configuration."

# Deploy the application
echo "Deploying application to Railway..."
railway up

if [ $? -eq 0 ]; then
    echo "Deployment successful!"
    echo "Your application is now deployed to Railway."
    echo "You can view your deployment at: $(railway domain)"
else
    echo "Deployment failed. Please check the logs with 'railway logs'."
    exit 1
fi

echo "Deployment process completed."