import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_aerostack_sim = get_package_share_directory('aerostack_sim')
    
    # Path to PX4 Autopilot (Outside src)
    px4_dir = os.path.expanduser('~/workspace/px4/PX4-Autopilot')
    
    # Simulation Arguments
    world_name = LaunchConfiguration('world', default='aerostack_world')
    
    # 0. Set Environments
    # Update GZ_SIM_RESOURCE_PATH for our world/models
    # Note: In a real launch we'd combine with existing. Here we set for the session.
    resource_path = os.path.join(pkg_aerostack_sim, 'models') + ':' + os.path.join(pkg_aerostack_sim, 'worlds')
    set_gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path)

    # 1. Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_name}.sdf'}.items(),
    )

    # 2. Micro-XRCE-DDS Agent (UDP 8888)
    # Assuming the agent is installed in system path as per Dockerfile
    uxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # 3. PX4 SITL
    # px4_dir/build/px4_sitl_default/bin/px4 -d px4_dir/ROMFS/px4fmu_common -s etc/init.d-posix/rcS
    px4_sitl = ExecuteProcess(
        cmd=[
            os.path.join(px4_dir, 'build/px4_sitl_default/bin/px4'),
            '-d', os.path.join(px4_dir, 'ROMFS/px4fmu_common'),
            '-s', 'etc/init.d-posix/rcS',
            '-i', '0' # Instance 0
        ],
        cwd=px4_dir,
        output='screen'
    )

    # 4. ros_gz_bridge (Declarative)
    bridge_config = os.path.join(pkg_aerostack_sim, 'config', 'ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 5. AeroStack FCU Bridge (Normalization Node)
    fcu_bridge = Node(
        package='aerostack_fcu_bridge',
        executable='bridge_node',
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        uxrce_agent,
        px4_sitl,
        ros_gz_bridge,
        fcu_bridge
    ])
