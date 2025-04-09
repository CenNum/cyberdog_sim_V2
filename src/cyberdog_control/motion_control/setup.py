from setuptools import setup
from glob import glob
import os

package_name = 'motion_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cennum',
    maintainer_email='2874dsl@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cyberdog_walk = motion_control.cyberdog_walk:main',
            'custom_walk = motion_control.custom_walk:main',
            'stage1 = motion_control.stage1:main',
            'action_motion_server = motion_control.action_motion_server:main',
            'action_motion_client = motion_control.action_motion_client:main',
        ],
    },
)