#!/bin/bash
source /home/ratan/miniconda3/bin/activate aerostack_rl
cd /mnt/c/Users/ratan/Desktop/AeroStack-RL

echo "Starting MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 > /tmp/xrce_agent.log 2>&1 &
AGENT_PID=$!

sleep 5

echo "Starting PX4 SITL..."
cd px4/PX4-Autopilot
./build/px4_sitl_default/bin/px4 -d ROMFS/px4fmu_common -s etc/init.d-posix/rcS -i 0 > /tmp/px4_sitl.log 2>&1 &
PX4_PID=$!

echo "Waiting for connectivity..."
sleep 20

echo "Checking ROS 2 topics..."
ros2 topic list -t | grep fmu

echo "Checking vehicle_odometry info..."
ros2 topic info --verbose /fmu/out/vehicle_odometry

echo "Checking vehicle_odometry hz..."
timeout 5 ros2 topic hz /fmu/out/vehicle_odometry

# Cleanup
# kill $AGENT_PID $PX4_PID
