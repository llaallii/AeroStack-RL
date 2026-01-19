# AeroStack-RL

AeroStack-RL is a ROS 2 based UAV compute and autonomy experimentation platform. It connects embedded control nodes with high-level onboard computing to enable reliable autonomy and reinforcement-learning research.

## Project Structure

- `src/aerostack_interfaces`: Custom ROS 2 messages and services.
- `src/aerostack_supervisor`: System health monitoring and state machine.
- `src/aerostack_rl`: Reinforcement Learning integration (Gym wrapper, Policy Runner).
- `src/aerostack_sim`: Simulation-in-the-Loop (SITL) assets.
- `src/aerostack_bringup`: Launch files for system bringup.
- `scripts/`: Helper scripts for setup and maintenance.

## Build Instructions

```bash
colcon build --symlink-install
source install/setup.bash
```
