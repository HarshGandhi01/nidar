from setuptools import find_packages, setup

package_name = 'nidar_rescueswarm'

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
            'area_partitioner = nidar_rescueswarm.area_partitioner:main',
            'aerial_survivor_detector = nidar_rescueswarm.aerial_survivor_detector:main',
            'payload_delivery_controller = nidar_rescueswarm.payload_delivery_controller:main',
        ],
    },
)
