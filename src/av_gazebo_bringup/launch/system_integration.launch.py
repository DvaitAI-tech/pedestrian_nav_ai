from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

def generate_launch_description():
    # ---------------- PATHS ----------------
    pkg_gazebo_bringup = get_package_share_directory('av_gazebo_bringup')
    
    # Set package root so SDF file://materials/... resolves
    set_gz_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=pkg_gazebo_bringup + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )

    world_file = os.path.join(pkg_gazebo_bringup, 'worlds', 'av_test.world')
    xacro_file = os.path.join(pkg_gazebo_bringup, 'urdf', 'ego_vehicle.urdf.xacro')
    tmp_urdf = '/tmp/ego_vehicle.urdf'

    # ---------------- PROCESS XACRO ----------------
    # Expand xacro to URDF
    try:
        subprocess.run(['xacro', xacro_file, '-o', tmp_urdf], check=True)
    except Exception as e:
        print(f"Xacro conversion failed: {e}")

    # Read the URDF for Robot State Publisher
    with open(tmp_urdf, 'r') as infp:
        robot_desc = infp.read()

    # ---------------- NODES ----------------

    # 1. Gazebo Simulation
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, 
             '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 2. Robot State Publisher (TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. Spawn Vehicle (Delayed)
    spawn_vehicle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ego_vehicle', 
                   '-file', tmp_urdf, 
                   '-x', '-302', 
                   '-y', '0', 
                   '-z', '0.2', 
                   '-Y', '-1.60'],
        output='screen'
    )
    # Wait 5 seconds for Gazebo to load before spawning
    delayed_spawn_vehicle = TimerAction(period=5.0, actions=[spawn_vehicle])

    # 4. YOLO Detection Node
    # This assumes 'av_sensor_package' is built and sourced
    yolo_node = Node(
        package='av_sensor_package',
        executable='yolo_node',
        output='screen',
        parameters=[{'debug_view': True}]  # Set to True to see the popup window
    )

    # 5. Pedestrian Randomizer Node
    pedestrian_manager = Node(
        package='av_gazebo_bringup',
        executable='ped_randomizer.py', # Name of the script we installed
        output='screen'
    )
    # 6. Pedestrian tracking node
    tracking_manager = Node(
        package='av_sensor_package',
        executable='pedestrian_tracker_node', # Name of the script we installed
        output='screen'
    )
    # 7. Fusion Node
    geometry_fusion_node = Node(
        package='av_sensor_package',
        executable='pedestrian_world_projection', # Or 'lidar_camera_fusion' if you renamed it
        output='screen',
        parameters=[{'use_sim_time': True}]  # <--- THIS IS THE CRITICAL FIX
    )

    # 8. Pedestrian TTC Risk Node
    # Equivalent to: ros2 run av_sensor_package pedestrian_ttc_risk_world
    ttc_risk_node = Node(
        package='av_sensor_package',
        executable='pedestrian_ttc_risk_world',
        output='screen'
    )

    # 9. Fusion Visualizer Node
    # Equivalent to: ros2 run av_sensor_package fusion_visualizer
    fusion_visualizer_node = Node(
        package='risk_assesment',
        executable='fusion_visualizer',
        output='screen'
    )
    control_arbiter = Node(
        package='risk_assesment',
        executable='control_arbiter',
        output='screen'
    )
    safety_decision_node = Node(
        package='risk_assesment',
        executable='safety_decision_node',
        output='screen'
    )
    pedestrian_bev_fusion = Node(
        package='av_sensor_package',
        executable='pedestrian_bev_fusion',
        output='screen'
    )
    return LaunchDescription([
        set_gz_resource,
        gazebo,
        robot_state_publisher,
        delayed_spawn_vehicle,
        yolo_node,
        # pedestrian_manager,
        tracking_manager,
        geometry_fusion_node,
        ttc_risk_node,
        fusion_visualizer_node,
        control_arbiter,
        safety_decision_node,
        pedestrian_bev_fusion,
    ])