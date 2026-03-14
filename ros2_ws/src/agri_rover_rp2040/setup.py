from setuptools import find_packages, setup

package_name = 'agri_rover_rp2040'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rp2040.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='AgriRover',
    maintainer_email='dev@agrirover.com',
    description='RP2040 USB serial bridge node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rp2040_bridge = agri_rover_rp2040.rp2040_bridge:main',
        ],
    },
)
