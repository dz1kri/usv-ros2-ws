from setuptools import setup
import os
from glob import glob

package_name = 'usv_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dzikri',
    maintainer_email='dzikri@example.com',
    description='USV Control Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
	'autonomous_control = usv_control.autonomous_control:main',
        'teleop_keyboard = usv_control.teleop_keyboard:main',
	'mavros_seano_bridge = usv_control.mavros_seano_bridge:main',
	'seano_mission_executor = usv_control.seano_mission_executor:main',
	'gz_waypoint_navigator = usv_control.gz_waypoint_navigator:main',
        ],
    },
)
