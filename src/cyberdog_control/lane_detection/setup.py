from setuptools import setup
from glob import glob
import os

package_name = 'lane_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*')))
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
            'qrscan = lane_detection.qrscan:main',
            'lanemaintain_control = lane_detection.lanemaintain_control:main'
        ],
    },
)
