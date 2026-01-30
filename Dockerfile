FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ---------- System deps (rarely change) ----------
RUN apt update && apt install -y \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    libglvnd0 \
    libglvnd-dev \
    libgl1 \
    libegl1 \
    libx11-6 \
    libxext6 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libxi6 \
    mesa-utils \
    git \
    curl \
    && apt remove -y python3-sympy \
    && rm -rf /var/lib/apt/lists/*

# ---------- Python deps (heavy, but stable) ----------
RUN pip3 install --upgrade pip
RUN pip3 install ultralytics

# ---------- Workspace (changes often) ----------
WORKDIR /home/nk/pedestrian_nav_ai

CMD ["/bin/bash"]
