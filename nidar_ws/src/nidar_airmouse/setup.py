from setuptools import find_packages, setup

package_name = 'nidar_airmouse'

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
    maintainer='harsh',
    maintainer_email='harsh.gandhi908@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'survivor_detector_node = nidar_airmouse.survivor_detector_node:main',
            'maze_explorer_node = nidar_airmouse.maze_explorer_node:main',
            'mission_manager_node = nidar_airmouse.mission_manager_node:main',
            'takeoff_and_fly = nidar_airmouse.takeoff_and_fly:main',
        ],
    },
)
