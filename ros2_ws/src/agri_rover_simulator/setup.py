from setuptools import find_packages, setup

package_name = 'agri_rover_simulator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulator.launch.py']),
        ('share/' + package_name + '/config', ['config/simulator_params.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='AgriRover',
    maintainer_email='dev@agrirover.com',
    description='Dead-reckoning GPS simulator for AgriRover development',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = agri_rover_simulator.simulator_node:main',
        ],
    },
)
