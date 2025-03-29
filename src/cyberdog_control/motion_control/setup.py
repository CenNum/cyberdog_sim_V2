from setuptools import setup

package_name = 'motion_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start_motion.launch.py']),
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
            'custom_walk = motion_control.custom_walk:main'
        ],
    },
)
