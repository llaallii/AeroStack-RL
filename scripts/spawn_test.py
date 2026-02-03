import subprocess
import json

def spawn():
    cmd = [
        "gz", "service", "-s", "/world/aerostack_world/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000",
        "--req", 'sdf_filename: "aerostack_drone", name: "manual_drone"'
    ]
    env = {
        "GZ_SIM_RESOURCE_PATH": "/home/ros/workspace/install/aerostack_sim/share/aerostack_sim/models:/home/ros/workspace/install/aerostack_sim/share/aerostack_sim/worlds:/home/ros/workspace/px4/PX4-Autopilot/Tools/simulation/gz/models"
    }
    result = subprocess.run(cmd, env=env, capture_output=True, text=True)
    print("STDOUT:", result.stdout)
    print("STDERR:", result.stderr)
    print("Exit Code:", result.returncode)

if __name__ == "__main__":
    spawn()
