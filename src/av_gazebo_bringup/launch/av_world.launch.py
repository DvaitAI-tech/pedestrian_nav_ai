from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

def generate_launch_description():
    pkg_share = get_package_share_directory('av_gazebo_bringup')

    # Set package root so SDF file://materials/... resolves
    set_gz_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=pkg_share + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )

    world_file = os.path.join(pkg_share, 'worlds', 'av_test.world')

    # Generate URDF from xacro to a temp file
    xacro_file = os.path.join(pkg_share, 'urdf', 'ego_vehicle.urdf.xacro')
    tmp_urdf = '/tmp/ego_vehicle.urdf'
    
    # expand xacro (works if xacro installed)
    try:
        subprocess.run(['xacro', xacro_file, '-o', tmp_urdf], check=True)
    except Exception as e:
        print(f"Xacro conversion failed: {e}")

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Robot State Publisher (REQUIRED for Transforms/TF)
    # Reads the file we just generated
    with open(tmp_urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Spawn ego at runway start (x = -450), y=0, z small lift
    spawn_ego = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ego_vehicle', 
                   '-file', tmp_urdf, 
                   '-x', '0', 
                   '-y', '0', 
                   '-z', '0.2', 
                   '-Y', '0.0'],  # Changed to 0.0 (Forward)
        output='screen'
    )

    # === THE FIX: Add a 5-second timer before spawning ===
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_ego]
    )

    return LaunchDescription([
        set_gz_resource,
        gazebo,
        robot_state_publisher, # Added RSP so your TF tree works
        delayed_spawn          # Used the delayed version of spawn
    ])