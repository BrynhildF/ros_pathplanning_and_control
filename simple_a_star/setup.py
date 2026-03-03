import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'simple_a_star'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='w123',
    maintainer_email='w123@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
       		'map_pub = simple_a_star.map_pub:main',
        	'a_star = simple_a_star.a_star:main',
        	'pid_controller = simple_a_star.pid_controller:main',
            	'robot_sim = simple_a_star.robot_sim:main',
            	'visualizer = simple_a_star.visualizer:main',
        ],
    },
)
