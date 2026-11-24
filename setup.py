from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'my_first_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'pkg_tools', 'launch'), glob('pkg_tools/launch/*.launch.py')),
        (os.path.join('share', package_name, 'pkg_tools', 'config'), glob('pkg_tools/config/*')),
        (os.path.join('share', package_name, 'pkg_tools', 'gazebo'), glob('pkg_tools/gazebo/*')),
        (os.path.join('share', package_name, 'pkg_tools', 'map'), glob('pkg_tools/map/*')),
        (os.path.join('share', package_name, 'pkg_tools', 'param'), glob('pkg_tools/param/*')),
        (os.path.join('share', package_name, 'pkg_tools', 'rviz'), glob('pkg_tools/rviz/*.rviz')),
        (os.path.join('share', package_name, 'pkg_tools', 'urdf'), glob('pkg_tools/urdf/*')),
        (os.path.join('share', package_name, 'pkg_tools', 'worlds'), glob('pkg_tools/worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='lee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_parking = my_first_pkg.aruco_parking:main',
            'aruco_tracker = my_first_pkg.aruco_tracker:main',
            'go_to_pose = my_first_pkg.go_to_pose:main',
            'core = my_first_pkg.core:main',
            'kit_moveit_config = my_first_pkg.kit_moveit_config:main',
            'moveit_py_demo = my_first_pkg.moveit_py_demo:main',
        ],
    },
)
