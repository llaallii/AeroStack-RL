#!/bin/bash
# verify_sim.sh - Verify AeroStack simulation health

echo "Verifying Simulation Health..."

# 1. Check for PX4 DDS Topics
echo -n "Checking PX4 Telemetry topics... "
TOPICS=$(ros2 topic list | grep "/fmu/out")
if [ -n "$TOPICS" ]; then
    echo "[PASS]"
else
    echo "[FAIL] No PX4 DDS topics detected. Is the Agent running?"
    exit 1
fi

# 2. Check for Normalized UavState
echo -n "Checking AeroStack UavState... "
if ros2 topic list | grep -q "/uav1/uav_state"; then
    echo "[PASS]"
else
    echo "[FAIL] /uav1/uav_state not found."
    exit 1
fi

# 3. Check Sensor Rates (Hz)
echo "Checking Sensor Rates (target 10-30Hz)..."

check_rate() {
    TOPIC=$1
    echo -n "  $TOPIC: "
    RATE=$(ros2 topic hz $TOPIC -c 5 | grep "average rate" | awk '{print $4}' | cut -d'.' -f1)
    if [ -n "$RATE" ] && [ "$RATE" -gt 0 ]; then
        echo "[$RATE Hz]"
    else
        echo "[NO DATA]"
    fi
}

check_rate "/uav1/scan"
check_rate "/uav1/camera/image_raw"
check_rate "/uav1/range"

echo "Simulation Verification Complete."
