#!/bin/bash
echo "Setting up AeroStack-RL environment..."
echo "Checking host environment..."

# 1. Check for WSLg
if [ -d "/mnt/wslg" ]; then
    echo "[PASS] WSLg detected."
else
    echo "[WARN] /mnt/wslg not found. GUI applications may not work."
fi

# 2. Check for Docker
if command -v docker &> /dev/null; then
    echo "[PASS] Docker is installed."
else
    echo "[FAIL] Docker not found. Please install Docker Desktop."
    exit 1
fi

# 3. Check for NVIDIA GPU (Simple check via nvidia-smi on host)
if command -v nvidia-smi &> /dev/null; then
    echo "[PASS] NVIDIA GPU detected."
else
    echo "[WARN] nvidia-smi not found. Ensure NVIDIA drivers are installed if you need GPU acceleration."
fi

echo "Environment Setup Complete. Open this folder in VS Code and select 'Reopen in Container'."
