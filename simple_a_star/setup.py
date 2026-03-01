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
        ],
    },
)
