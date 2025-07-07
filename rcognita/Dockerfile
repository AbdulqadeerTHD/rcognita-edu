FROM python:3.9-slim

# Install system dependencies
RUN apt update && apt install -y git build-essential curl

# Set working directory
WORKDIR /app

# Copy everything
COPY . /app

# Upgrade pip and install Python requirements
RUN pip install --upgrade pip && \
    pip install -r requirements.txt && \
    pip install jupyter

# Expose Jupyter port
EXPOSE 8888

# Start Jupyter by default
CMD ["jupyter", "notebook", "--ip=0.0.0.0", "--port=8888", "--allow-root", "--no-browser"]

