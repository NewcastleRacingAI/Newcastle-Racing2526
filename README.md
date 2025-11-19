# Newcastle Racing 2025/2026

To start working on this project, clone the respository and ensure you are on the right branch.

```bash
git clone --recurse-submodules https://github.com/NewcastleRacingAI/Newcastle-Racing.git
cd Newcastle-Racing
git checkout 2526
git submodule update --init --recursive
```

## Project structure

This is the main workspace repository containing all the sub-packages that make up the Newcastle Racing AI project.

```bash
Newcastle-Racing
├── src # Ros packages
│   ├── zed-ros2-wrapper # Ready
│   ├── eufs_msgs # Ready
│   ├── ros_can # Ready
│   ├── nrai_odometry # TODO
│   ├── nrai_perception # TODO
│   ├── nrai_path_planning # 1  - TODO
│   ├── ft-fsd-path-planning # 1 
│   ├── nrai_controller # TODO
│   └── nrai_mission_control # TODO
└── hello_world.sh # Entry point script
```

## Running the full ROS workspace

First, install the ROS dependencies with `rosdep`

```bash
# Initialize the rosdep package manager (it may need sudo)
rosdep init || echo "rosdep already initialized"
# Update rosdep
rosdep update
# Install the ros dependencies
rosdep install --from-paths src --ignore-src -r -y
```

Then, build the ROS packages

```bash
/opt/ros/humble/setup.sh
colcon build --symlink-install
```

