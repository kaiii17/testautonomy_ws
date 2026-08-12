from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'boat_slam_bringup'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='kbm11',
    maintainer_email='kbm11@example.com',

    description='Cartographer mapping and localization for boat',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'boat_marker = boat_slam_bringup.boat_marker:main',
            'gap_path_visualizer = boat_slam_bringup.gap_path_visualizer:main',
        ],
    },
)
