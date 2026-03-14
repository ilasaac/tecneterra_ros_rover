from setuptools import find_packages, setup

package_name = 'agri_rover_video'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/video.launch.py']),
        ('share/' + package_name + '/config', ['config/gstreamer.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AgriRover',
    maintainer_email='dev@agrirover.com',
    description='Video streaming node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_streamer = agri_rover_video.video_streamer:main',
        ],
    },
)
