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

# Copy requirements and install
COPY --chown=user requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy everything
COPY --chown=user . .

# Install backend specific requirements
RUN if [ -f "backend/requirements.txt" ]; then pip install --no-cache-dir -r backend/requirements.txt; fi

# IMPORTANT: Change directory to backend folder so 'src' is a top-level package
WORKDIR $HOME/app/backend

# Set Python path
ENV PYTHONPATH=$HOME/app/backend

# Expose port 7860 for Hugging Face
EXPOSE 7860

# Run uvicorn directly
CMD ["python", "-m", "uvicorn", "src.api.main:app", "--host", "0.0.0.0", "--port", "7860"]
