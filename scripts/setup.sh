#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}Verifying AeroStack-RL Native WSL Environment...${NC}"

# 1. Check for WSLg
if [ -d "/mnt/wslg" ]; then
    echo -e "${GREEN}[PASS] WSLg detected.${NC}"
else
    echo -e "${RED}[WARN] /mnt/wslg not found. GUI applications may not work.${NC}"
fi

# 2. Check for NVIDIA GPU
if command -v nvidia-smi &> /dev/null; then
    echo -e "${GREEN}[PASS] NVIDIA GPU detected.${NC}"
else
    echo -e "${RED}[WARN] nvidia-smi not found. GPU acceleration might be limited.${NC}"
fi

# 3. Check for ROS 2 Jazzy
if command -v ros2 &> /dev/null; then
    ROS_VERSION=$(ros2 --version | head -n 1)
    echo -e "${GREEN}[PASS] ROS 2 detected: $ROS_VERSION${NC}"
else
    echo -e "${RED}[FAIL] ROS 2 not found. Run ./scripts/setup_wsl.sh first.${NC}"
fi

# 4. Check for Gazebo Harmonic
if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version | head -n 1)
    echo -e "${GREEN}[PASS] Gazebo Sim detected: $GZ_VERSION${NC}"
else
    echo -e "${RED}[FAIL] Gazebo Sim not found. Run ./scripts/setup_wsl.sh first.${NC}"
fi

# 5. Check for Conda Environment
if command -v conda &> /dev/null; then
    if conda env list | grep -q "aerostack_rl"; then
        echo -e "${GREEN}[PASS] Conda environment 'aerostack_rl' found.${NC}"
    else
        echo -e "${RED}[FAIL] Conda environment 'aerostack_rl' not found.${NC}"
    fi
else
    echo -e "${RED}[FAIL] Conda not installed.${NC}"
fi

echo -e "\n${BLUE}To start the environment:${NC}"
echo -e "1. source ~/.bashrc"
echo -e "2. conda activate aerostack_rl"
echo -e "3. ros2 launch aerostack_sim sitl.launch.py"
