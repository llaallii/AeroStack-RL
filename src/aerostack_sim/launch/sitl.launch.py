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
    
    # Path to PX4 Autopilot (Relative to project root)
    # The script is run from project root, so we find where PX4 is
    current_dir = os.getcwd()
    px4_dir = os.path.join(current_dir, 'px4/PX4-Autopilot')
    
    # Simulation Arguments
    world_name = LaunchConfiguration('world', default='aerostack_world')
    
    # 0. Set Environments
    # Combine resource paths: Aerostack models + PX4 models
    px4_models_dir = os.path.join(px4_dir, 'Tools/simulation/gz/models')
    resource_path = (
        os.path.join(pkg_aerostack_sim, 'models') + ':' + 
        os.path.join(pkg_aerostack_sim, 'worlds') + ':' +
        px4_models_dir
    )
    
    pkg_fcu_bridge = get_package_share_directory('aerostack_fcu_bridge')
    # Add site-packages to PYTHONPATH (Ubuntu 24.04 uses Python 3.12)
    site_packages = os.path.join(os.path.dirname(os.path.dirname(pkg_fcu_bridge)), 'lib/python3.12/site-packages')
    
    # Start with current process env to preserve ROS 2 and Gazebo paths
    full_env = os.environ.copy()
    full_env['GZ_SIM_RESOURCE_PATH'] = resource_path
    full_env['PYTHONPATH'] = site_packages + ':' + full_env.get('PYTHONPATH', '')
    # Ensure standard bins are in PATH for PX4 scripts
    full_env['PATH'] = full_env.get('PATH', '') + ':' + '/usr/bin' + ':' + '/bin' + ':' + '/usr/local/bin'
    
    # GUI/Rendering environment
    full_env['DISPLAY'] = os.environ.get('DISPLAY', ':0')
    full_env['GZ_SIM_RENDER_ENGINE_GUESS'] = 'ogre2' # Force Ogre2 for GPU sensors
    
    # Ensure Gazebo version is set for PX4
    full_env['GZ_VERSION'] = '8'

    # 1. Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            '-v 4 -r ', world_name, '.sdf'
        ]}.items(),
    )
    # Clock bridge (Gazebo -> ROS)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    # Set it as an action too as some nodes might need it via lookup
    set_gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path)

    # 2. Micro-XRCE-DDS Agent (UDP 8888)
    uxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        env=full_env
    )

    # 3. Spawner
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'aerostack_world',
            '-file', os.path.join(pkg_aerostack_sim, 'models', 'aerostack_drone', 'model.sdf'),
            '-name', 'aerostack_drone',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen',
        env=full_env
    )

    # 4. PX4 SITL
    px4_env = full_env.copy()
    px4_env.update({
        'PX4_SYS_AUTOSTART': '4001', # x500
        'PX4_GZ_MODEL_NAME': 'aerostack_drone', # Keep this so PX4 knows the model name to connect to
        'PX4_GZ_WORLD': 'aerostack_world',
        'PX4_GZ_WORLDS': os.path.join(pkg_aerostack_sim, 'worlds')
    })
    
    px4_sitl = ExecuteProcess(
        cmd=[
            os.path.join(px4_dir, 'build/px4_sitl_default/bin/px4'),
            '-d', os.path.join(px4_dir, 'ROMFS/px4fmu_common'),
            '-s', 'etc/init.d-posix/rcS',
            '-i', '0'
        ],
        cwd=px4_dir,
        env=px4_env,
        output='screen'
    )

    # 5. ros_gz_bridge (Declarative)
    bridge_config = os.path.join(pkg_aerostack_sim, 'config', 'ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
        env=full_env
    )

    # 6. AeroStack FCU Bridge (Normalization Node)
    fcu_bridge_bin = os.path.join(os.path.dirname(os.path.dirname(pkg_fcu_bridge)), 'bin', 'bridge_node')
    fcu_bridge = ExecuteProcess(
        cmd=[fcu_bridge_bin],
        env=full_env,
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        spawner,
        clock_bridge,
        uxrce_agent,
        px4_sitl,
        ros_gz_bridge,
        fcu_bridge
    ])
