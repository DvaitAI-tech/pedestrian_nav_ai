from setuptools import find_packages, setup

package_name = 'risk_assesment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nk',
    maintainer_email='nk.dvaitai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fusion_visualizer=risk_assesment.fusion_visualizer:main',
            'safety_decision_node=risk_assesment.safety_decision_node:main',
            'control_arbiter=risk_assesment.control_arbiter:main',
        ],
    },
)
