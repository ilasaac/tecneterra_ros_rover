from setuptools import find_packages, setup

package_name = 'agri_rover_navigator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigator.launch.py']),
        ('share/' + package_name + '/config', ['config/navigator_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AgriRover',
    maintainer_email='dev@agrirover.com',
    description='Autonomous navigator node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = agri_rover_navigator.navigator:main',
        ],
    },
)
