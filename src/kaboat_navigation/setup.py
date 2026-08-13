from setuptools import find_packages, setup

package_name = 'kaboat_navigation'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mission_manager = kaboat_navigation.mission_manager:main',
            'test_mission_manager = kaboat_navigation.test_mission_manager:main',
            'arbiter = kaboat_navigation.arbiter:main',
            'avoidance = kaboat_navigation.avoidance:main',
            'mission_1 = kaboat_navigation.mission_1:main',
            'mission_2 = kaboat_navigation.mission_2:main',
            'mission_3 = kaboat_navigation.mission_3:main',
            'mission_4 = kaboat_navigation.mission_4:main',
            'mission_5 = kaboat_navigation.mission_5:main',
            'thruster_output = kaboat_navigation.thruster_output:main',
        ],
    },
)
