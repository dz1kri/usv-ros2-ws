from setuptools import setup

package_name = 'seano_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dzikri',
    maintainer_email='dzikri@example.com',
    description='SEANO Cloud Bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge = seano_bridge.seano_cloud_bridge:main',
	    'gz_pose = seano_bridge.gz_dynamic_pose_bridge:main',
        ],
    },
)
