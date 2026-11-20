from glob import glob

from setuptools import setup

package_name = 'tins8_mission'

setup(
    name=package_name,
    version='1.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='COE 510 student',
    maintainer_email='student@example.com',
    description='TinS-12E 8-UAV course build: mission, control, perception, pursuit, CLI and GUI.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carousel_supervisor = tins8_mission.carousel_supervisor:main',
            'uav_agent = tins8_mission.uav_agent:main',
            'yolox_detector = tins8_mission.yolox_detector:main',
            'target_tracker = tins8_mission.target_tracker:main',
            'pursuit_node = tins8_mission.pursuit_node:main',
            'mission_server = tins8_mission.mission_server:main',
            'pedestrian_driver = tins8_mission.pedestrian_driver:main',
            'fleet_manager = tins8_mission.fleet_manager:main',
            'evaluator = tins8_mission.evaluator:main',
            'gz_relay = tins8_mission.gz_relay:main',
            'tins8ctl = tins8_mission.cli:main',
            'tins8_gui = tins8_mission.gui:main',
        ],
    },
)
