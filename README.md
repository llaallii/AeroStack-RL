# AeroStack-RL

AeroStack-RL is a ROS 2 based UAV compute and autonomy experimentation platform. It connects embedded control nodes with high-level onboard computing to enable reliable autonomy and reinforcement-learning research.

## Project Structure

- `src/aerostack_interfaces`: Custom ROS 2 messages and services.
- `src/aerostack_supervisor`: System health monitoring and state machine.
- `src/aerostack_fcu_bridge`: MAVLink bridge for FCU communication.
- `src/aerostack_rl`: Reinforcement Learning integration (Gym wrapper, Policy Runner).
- `src/aerostack_sim`: Simulation-in-the-Loop (SITL) assets.
- `src/aerostack_bringup`: Launch files for system bringup.
- `scripts/`: Helper scripts for setup and maintenance.
- `retired/`: Archive of unneeded or legacy files and folders.

## Quick Start (Native WSL2)

1. **Prerequisites**: Ensure you have [Miniconda](https://docs.anaconda.com/miniconda/) installed in WSL (Ubuntu 24.04).
   - **Important**: Use **Python 3.12** in your conda environment to match ROS 2 Jazzy.
2. **Initial Setup**: Run the setup script to install ROS 2 Jazzy, Gazebo Harmonic, and RL dependencies:
   ```bash
   chmod +x scripts/*.sh
   ./scripts/setup_wsl.sh
   ```
3. **Verify Environment**:
   ```bash
   ./scripts/setup.sh
   ```
4. **Build Project**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
5. **Run Simulation**:
   ```bash
   # Activation automatically sources ROS 2 and the workspace!
   conda activate aerostack_rl
   ros2 launch aerostack_sim sitl.launch.py
   ```
