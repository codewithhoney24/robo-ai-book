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

# Copy requirements from root and backend
COPY --chown=user requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy everything into the container
COPY --chown=user . .

# Install backend specific requirements if any
RUN if [ -f "backend/requirements.txt" ]; then pip install --no-cache-dir -r backend/requirements.txt; fi

# Set Python path to include backend
ENV PYTHONPATH=$HOME/app:$HOME/app/backend

# Expose port
EXPOSE 7860

# CMD to run the backend server correctly
# We point to backend/server.py because that's where the entry is
CMD ["python", "backend/server.py"]
