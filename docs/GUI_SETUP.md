# GUI Setup Guide (VcXsrv)

Since native WSLg is not working on your host, we will use **VcXsrv**, a Windows X Server, to display Gazebo.

## 1. Install VcXsrv
1.  Download **VcXsrv** (available as "VcXsrv Windows X Server" on SourceForge).
2.  Install with default options.

## 2. Configure VcXsrv (Launch XLaunch)
1.  Open **XLaunch** from your Start Menu.
2.  **Display Settings**: Select "Multiple windows", Display number `-1`. Click Next.
3.  **Client Startup**: Select "Start no client". Click Next.
4.  **Extra Settings**:
    *   [x] **Clipboard**
    *   [x] **Native opengl**
    *   [x] **Disable access control** (Critical! This allows Docker to connect).
5.  Click Next, then **Finish**.

> **Tip**: You can "Save configuration" to a `.xlaunch` file on your Desktop to double-click next time.

## 3. Configure Container
I have updated the Dockerfile to automatically point the display to your Windows host.
1.  **Rebuild Container**: `Dev Containers: Rebuild and Reopen`.
2.  **Verify**: Run `echo $DISPLAY` inside the terminal. It should show an IP address ending in `:0`.
3.  **Test**: Run `xeyes` (if installed) or `gz sim`.

## Troubleshooting
*   **Firewall**: Ensure VcXsrv is allowed through the Windows Firewall (Public and Private networks).
*   **Black Screen**: Try unchecking "Native opengl" in XLaunch if 3D rendering fails.
