# GUI Setup Guide

AeroStack-RL uses Gazebo Harmonic for simulation. In a WSL2 environment, GPU-accelerated GUI performance is critical.

## 1. Native WSLg (Recommended)
If you are on Windows 11 or updated Windows 10, **WSLg** provides native high-performance GUI support.

### Prerequisites
- Latest **NVIDIA Drivers** installed on Windows.
- WSL version 2.

### Verification
In your WSL terminal:
```bash
# Check if hardware acceleration is active (should show NVIDIA)
glxinfo -B | grep "OpenGL renderer"
```

If it says `llvmpipe`, you are using software rendering. Ensure your Windows NVIDIA drivers are up to date.

---

## 2. VcXsrv (Legacy/Fallback)
If native WSLg is not working, you can use **VcXsrv** as a fallback X Server.

### Installation & Configuration
1.  Download and install **VcXsrv**.
2.  Launch **XLaunch**.
3.  Choose **Multiple windows**.
4.  Check **Native opengl**.
5.  Check **Disable access control** (Critical).
6.  In WSL, export your display:
    ```bash
    export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
    ```

---

## 3. High-DPI Scaling (Optional)
If the Gazebo UI is too small:
1.  Right-click `wslg.exe` (or your terminal).
2.  Go to Properties -> Compatibility -> Change high DPI settings.
3.  Override high DPI scaling behavior (System or System Enhanced).
