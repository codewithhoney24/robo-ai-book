#!/bin/bash
# ROS 2 Humble Setup Script

echo "Setting up ROS 2 Humble development environment for content validation..."

# Check if running on Ubuntu
if [ ! -f /etc/os-release ]; then
    echo "Error: This script is designed for Ubuntu systems"
    exit 1
fi

. /etc/os-release
if [[ "$NAME" != "Ubuntu" ]] || [[ "$VERSION_ID" != "22.04" ]]; then
    echo "Warning: This script is designed for Ubuntu 22.04 LTS"
fi

# Add the ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/rosSigning.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2

# Initialize rosdep
sudo rosdep init
rosdep update

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "ROS 2 Humble setup complete!"
echo "Please run 'source ~/.bashrc' or restart your terminal to use ROS 2"