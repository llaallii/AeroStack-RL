#!/bin/bash
set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

echo -e "${BLUE}Starting AeroStack-RL Native WSL Setup...${NC}"
echo -e "${BLUE}Project root: $PROJECT_ROOT${NC}"

# 1. Update and install basic dependencies
echo -e "${GREEN}Updating system and installing base dependencies...${NC}"
sudo apt update && sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-venv \
    mesa-utils \
    vulkan-tools

# 2. Install ROS 2 Jazzy
if ! command -v ros2 &> /dev/null; then
    echo -e "${GREEN}Installing ROS 2 Jazzy...${NC}"
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt install -y ros-jazzy-desktop ros-dev-tools
else
    echo -e "${BLUE}ROS 2 already installed.${NC}"
fi

# 3. Install Gazebo Harmonic
if ! command -v gz &> /dev/null; then
    echo -e "${GREEN}Installing Gazebo Harmonic...${NC}"
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt update && sudo apt install -y gz-harmonic \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-interfaces
else
    echo -e "${BLUE}Gazebo Harmonic already installed.${NC}"
fi

# 4. Setup Micro-XRCE-DDS Agent
if [ ! -f "/usr/local/bin/MicroXRCEAgent" ]; then
    echo -e "${GREEN}Building Micro-XRCE-DDS Agent...${NC}"
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent
    mkdir -p /tmp/Micro-XRCE-DDS-Agent/build && cd /tmp/Micro-XRCE-DDS-Agent/build
    cmake .. && make && sudo make install && sudo ldconfig
    rm -rf /tmp/Micro-XRCE-DDS-Agent
else
    echo -e "${BLUE}Micro-XRCE-DDS Agent already installed.${NC}"
fi

# 5. PX4 Toolchain and Build
echo -e "${GREEN}Setting up PX4 Toolchain...${NC}"
if [ -d "$PROJECT_ROOT/px4/PX4-Autopilot" ]; then
    # Run PX4's official setup script (non-interactive)
    bash "$PROJECT_ROOT/px4/PX4-Autopilot/Tools/setup/ubuntu.sh" --no-nuttx --no-sim-tools
    
    echo -e "${GREEN}Building PX4 SITL - this may take a while...${NC}"
    cd "$PROJECT_ROOT/px4/PX4-Autopilot"
    # make clean  # Removed to allow incremental/cached builds
    make px4_sitl_default
    cd ../..
else
    echo -e "${RED}[WARN] px4/PX4-Autopilot directory not found. Skipping PX4 build.${NC}"
fi

# 6. Conda Setup for RL
echo -e "${GREEN}Setting up Conda environment...${NC}"

# Robust conda detection
if command -v conda &> /dev/null; then
    CONDA_EXE=$(command -v conda)
elif [ -f "$HOME/miniconda3/bin/conda" ]; then
    CONDA_EXE="$HOME/miniconda3/bin/conda"
elif [ -f "$HOME/anaconda3/bin/conda" ]; then
    CONDA_EXE="$HOME/anaconda3/bin/conda"
else
    CONDA_EXE=""
fi

if [ -z "$CONDA_EXE" ]; then
    echo -e "${RED}Conda not found. Please install Miniconda manually and rerun this script.${NC}"
    echo "Visit: https://docs.anaconda.com/miniconda/"
else
    echo -e "${BLUE}Using conda at: $CONDA_EXE${NC}"
    # Get the base directory of conda to source the shell hook
    CONDA_BASE=$(dirname $(dirname "$CONDA_EXE"))
    source "$CONDA_BASE/etc/profile.d/conda.sh"

    # Delete environment if it exists for a fresh start
    if conda env list | grep -q "aerostack_rl"; then
        echo -e "${BLUE}Removing existing aerostack_rl environment for a fresh start...${NC}"
        conda remove -y -n aerostack_rl --all
    fi

    # Create environment
    echo -e "${GREEN}Creating aerostack_rl environment with Python 3.12...${NC}"
    conda create -y -n aerostack_rl python=3.12
    
    # Install dependencies
    conda run -n aerostack_rl pip install --upgrade pip
    conda run -n aerostack_rl pip install torch stable-baselines3 shimmy>=2.0.0 tensorboard
    conda run -n aerostack_rl pip install kconfiglib jinja2 jsonschema pyros-genmsg  # PX4 Build Deps
    conda run -n aerostack_rl pip install pre-commit black flake8 pytest empy==3.3.4 catkin_pkg lark setuptools

    # --- Conda Activation Hooks ---
    CONDA_PREFIX_PATH=$(conda env list | grep "aerostack_rl" | awk '{print $NF}')
    ACTIVATE_D="$CONDA_PREFIX_PATH/etc/conda/activate.d"
    mkdir -p "$ACTIVATE_D"

    echo -e "${GREEN}Creating Conda activation hooks for ROS 2...${NC}"
    cat <<EOF > "$ACTIVATE_D/ros_setup.sh"
#!/bin/bash
# Auto-source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Auto-source workspace if built
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
fi
EOF
fi

# 7. Final Cleanup and Instructions
echo -e "${GREEN}Cleaning up old build artifacts...${NC}"
rm -rf "$PROJECT_ROOT/build" "$PROJECT_ROOT/install" "$PROJECT_ROOT/log"

echo -e "${BLUE}Setup complete! Please restart your terminal or run:${NC}"
echo -e "  source ~/.bashrc"
echo -e "  conda activate aerostack_rl"
echo -e "  colcon build --symlink-install"
