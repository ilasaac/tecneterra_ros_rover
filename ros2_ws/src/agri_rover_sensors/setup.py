from setuptools import find_packages, setup

package_name = 'agri_rover_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensors.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AgriRover',
    maintainer_email='dev@agrirover.com',
    description='Agricultural sensor node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = agri_rover_sensors.sensor_node:main',
        ],
    },
)
