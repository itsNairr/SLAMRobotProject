from setuptools import find_packages, setup

package_name = 'odometry_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hari Nair',
    maintainer_email='21hpn2@queensu.ca',
    description='Odometry ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'odometry_node = odometry_ros.odometry_node:main'
        ],
    },
)
