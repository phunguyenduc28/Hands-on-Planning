# 1. Base Image: NVIDIA Ubuntu 24.04 with CUDA 13.1
FROM nvidia/cuda:13.1.1-devel-ubuntu24.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=jazzy

# NVIDIA Graphics Support Environment
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,display,utility

# 2. Install Basic Tools & Locales
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales sudo git build-essential cmake \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8

# 3. Add ROS 2 Jazzy Repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Install ROS Development Tools & Hardware Libraries
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    ros-jazzy-desktop \
    python3-colcon-common-extensions \
    libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y \
    ros-jazzy-teleop-twist-* \
    ros-jazzy-turtlebot4* \
    ros-jazzy-rviz2* \
    ros-jazzy-kobuki* \
    ros-jazzy-realsense* \
    ros-jazzy-octomap* \
    ros-jazzy-laser* \
    && rm -rf /var/lib/apt/lists/*

# 5. Install Stonefish Library Dependencies
RUN apt-get update && apt-get install -y \
    libglm-dev \
    libsdl2-dev \
    libfreetype6-dev \
    libglew-dev \
    libglu1-mesa-dev \
    pkg-config \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*
	
# Fix SDL2 cmake config (Known Stonefish requirement on some Ubuntu versions)
RUN sed -i 's/-lSDL2 /-lSDL2/g' /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake || true

# 6. Build Stonefish Library from Source
WORKDIR /opt
RUN git clone https://github.com/patrykcieslak/stonefish.git \
    && cd stonefish \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && sudo make install
    
# 7. Register /usr/local/lib so the system finds Stonefish without LD_LIBRARY_PATH
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/stonefish.conf && ldconfig

# 8. Force NVIDIA usage for OpenGL
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

# 9. Disable V-Sync (Crucial for simulation sync)
ENV __GL_VBLANK_MODE=0 

# 10. Optimized ROS 2 DDS Settings
# This helps prevent "Odom Lag" by using Shared Memory
COPY <<EOF /tmp/shm_linux.xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastrtps_profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>shm_transport</transport_id>
            <type>SHM</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="shm_part" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>shm_transport</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
EOF

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/shm_linux.xml

RUN apt-get update && apt-get install -y python3-pip
    
WORKDIR /root/ros2_ws

# 8. Setup Environment Sourcing
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
    
# glxinfo | grep OpenGL (apt install mesa-utils)

CMD ["bash"]
