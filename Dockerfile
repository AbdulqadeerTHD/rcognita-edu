# ===============================
#   DOCKERFILE FOR RCOGNITA
# ===============================

# --- 1. Base image with ROS Noetic core (Ubuntu 20.04) ---
FROM ros:noetic-ros-core

# --- 2. Environment variables ---
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# --- 3. System dependencies ---
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-matplotlib \
    python3-scipy \
    python3-tk \
    git \
    curl \
    nano \
    net-tools \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# --- 4. Python packages ---
RUN pip3 install --upgrade pip && \
    pip3 install \
    casadi \
    sippy \
    matplotlib \
    scipy \
    numpy==1.23.5 \
    pandas \
    opencv-python \
    jupyterlab \
    tabulate \
    svgpath2mpl

# --- 5. Set working directory ---
WORKDIR /home/rcognita_ws

# --- 6. Copy code into container ---
COPY . /home/rcognita_ws

# --- 7. Default command ---
CMD ["bash"]
