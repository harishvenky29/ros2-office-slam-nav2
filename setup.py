from glob import glob

from setuptools import find_packages, setup

package_name = 'office_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harish Venkatesan M R',
    maintainer_email='mrharishv29@gmail.com',
    description='TurtleBot3 LIDAR SLAM mapping, frontier exploration and Nav2 patrol in a small office.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'frontier_explorer = office_nav.frontier_explorer:main',
            'waypoint_patrol = office_nav.waypoint_patrol:main',
        ],
    },
)
