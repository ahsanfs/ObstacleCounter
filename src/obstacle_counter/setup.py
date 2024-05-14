from setuptools import setup

package_name = 'obstacle_counter'

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
    maintainer='san',
    maintainer_email='san@todo.todo',
    description='Obstacle counting node for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_counter = obstacle_counter.obstacle_counter:main',
        ],
    },
)
