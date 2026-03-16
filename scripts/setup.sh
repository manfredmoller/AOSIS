#!/bin/bash

# =================================================================
# AOSIS: Environment Setup Script
# This script restores the "ephemeral" container environment
# to a state ready for professional development.
# =================================================================

# 1. Update and Install System Tools
# We use -y to skip the "Do you want to continue?" prompts.
echo "Checking for system tools..."
sudo apt update && sudo apt install -y \
    tmux \
    htop \
    vim \
    bash-completion \
    python3-pip

# 2. Python Dependencies
# Add any specific Python libraries for your tracking logic here.
# pip3 install opencv-python-headless # Example if needed

# 3. Workspace Sourcing
# We add these to the .bashrc of the admin user so they
# persist across terminal tabs in this session.
echo "Configuring shell environment..."

# Check if we've already added these to avoid duplicates
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source /workspaces/isaac_ros-dev/install/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc
fi

# 4. Permissions
# Ensure the admin user owns the workspace folders
sudo chown -R admin:admin /workspaces/isaac_ros-dev/src/inspection_station

echo "================================================="
echo "AOSIS Setup Complete!"
echo "Run 'source ~/.bashrc' or open a new terminal."
echo "================================================="