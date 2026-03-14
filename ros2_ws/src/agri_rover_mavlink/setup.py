from setuptools import find_packages, setup

package_name = 'agri_rover_mavlink'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mavlink.launch.py']),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='AgriRover',
    maintainer_email='dev@agrirover.com',
    description='MAVLink ↔ ROS2 bridge node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavlink_bridge = agri_rover_mavlink.mavlink_bridge:main',
        ],
    },
)
