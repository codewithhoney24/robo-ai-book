# Use an official Python runtime as a parent image
FROM python:3.11-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1
ENV PORT=7860

# Set work directory
WORKDIR /app

# Install system dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        postgresql-client \
        build-essential \
        libffi-dev \
        libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user (Hugging Face Requirement)
RUN useradd -m -u 1000 user
USER user
ENV HOME=/home/user
ENV PATH=/home/user/.local/bin:$PATH
WORKDIR $HOME/app

# Copy ALL requirements files
COPY --chown=user requirements.txt .
COPY --chown=user backend/requirements.txt ./backend_requirements.txt

# Install all dependencies
RUN pip install --no-cache-dir -r requirements.txt
RUN pip install --no-cache-dir -r backend_requirements.txt

# Copy everything into the container
COPY --chown=user . .

# Set Python path to find src module inside backend folder
ENV PYTHONPATH=$HOME/app/backend

# Expose port
EXPOSE 7860

# CMD to run the backend server correctly
# Since we are in $HOME/app, we run uvicorn pointing to the module path
CMD ["python", "-m", "uvicorn", "backend.src.api.main:app", "--host", "0.0.0.0", "--port", "7860"]
