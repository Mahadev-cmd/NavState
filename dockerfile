FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rmw-fastrtps-cpp \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN python3 -m pip install --upgrade pip

# Install Python packages
RUN pip install \
    numpy==1.26.4 \
    opencv-python \
    ultralytics

# Set working directory
WORKDIR /workspace

# Copy your script into container
COPY video4.py .

# Set environment (optional)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Start with bash (interactive shell)
CMD ["/bin/bash"]
