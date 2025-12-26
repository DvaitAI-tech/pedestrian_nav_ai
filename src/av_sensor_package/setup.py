from setuptools import setup

package_name = 'av_sensor_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nk',
    maintainer_email='nk@example.com',
    description='AV sensor package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_node = av_sensor_package.yolo_node:main',
            'lidar_obstacle_node = av_sensor_package.obstacle_node:main',
            'pedestrian_tracker_node=av_sensor_package.pedestrian_tracker_node:main',
            'tracked_pedestrian_viz=av_sensor_package.tracked_pedestrian_viz:main',
            'tracked_pedestrian_plot=av_sensor_package.tracked_pedestrian_plot:main',
            'pedestrian_smoothing_prediction_plot=av_sensor_package.pedestrian_smoothing_prediction_plot:main',
            'pedestrian_ttc_risk_plot=av_sensor_package.pedestrian_ttc_risk_plot:main',
            'pedestrian_world_projection=av_sensor_package.pedestrian_world_projection:main',
            'pedestrian_ttc_risk_world=av_sensor_package.pedestrian_ttc_risk_world:main',
            'fusion_visualizer=av_sensor_package.fusion_visualizer:main'
        ],
    },
)
