from setuptools import setup

package_name = 'orb_slam3_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cennum',
    maintainer_email='cennum@todo.todo',
    description='ORB-SLAM3 ROS2 wrapper',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orb_slam3_node = orb_slam3_ros2.orb_slam3_node:main',
        ],
    },
)
