# Base image
FROM ros:humble

# Setup environment
ENV DEBIAN_FRONTEND=noninteractive
ENV EUFS_MASTER=/workspace

# Install GUI tools
RUN apt update && apt install -y \
    libyaml-cpp-dev \
    x11-apps \
    python3-colcon-common-extensions \
    python3-pip \
    python3-tk \
    python3-rosdep \
    nano \
    vim \
    wget


# Initialize rosdep
RUN rosdep update
RUN rosdep init || echo "rosdep already initialized"

# Create workspace root
WORKDIR /workspace

# Copy the ros packages (the ones we are not supposed to be actively developing)
COPY src/eufs_msgs ./src/eufs_msgs
COPY src/ros_can ./src/ros_can
COPY src/zed-ros2-wrapper ./src/zed-ros2-wrapper
# Install dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Copy the rest of the source code
COPY src ./src
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

