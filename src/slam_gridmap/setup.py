from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_gridmap'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Toia',
    maintainer_email='vlynch@udesa.edu.ar',
    description='Grid-Based FastSLAM (Parte A, Opcion 1) - TP Final I-402.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_fastslam = slam_gridmap.grid_fastslam_node:main',
        ],
    },
)
