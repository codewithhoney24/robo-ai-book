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

# Create a non-root user for Hugging Face security
RUN useradd -m -u 1000 user
USER user
ENV HOME=/home/user
ENV PATH=/home/user/.local/bin:$PATH
WORKDIR $HOME/app

# Copy requirements and install dependencies
COPY --chown=user requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the backend source code
COPY --chown=user . .

# Set Python path to include current directory
ENV PYTHONPATH=$HOME/app

# Set Hugging Face Spaces port
EXPOSE 7860

# Command to run the FastAPI server directly with uvicorn
CMD ["python", "-m", "uvicorn", "src.api.main:app", "--host", "0.0.0.0", "--port", "7860"]
