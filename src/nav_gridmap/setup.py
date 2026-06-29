from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_gridmap'

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
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.pgm') + glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Toia',
    maintainer_email='vlynch@udesa.edu.ar',
    description='Navegacion Autonoma sobre grilla pura (Parte B) - TP Final I-402.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Parte B - Localizacion (MCL contra el mapa de la Parte A)
            'mcl_localization = nav_gridmap.mcl_node:main',
            # Parte B - Navegador (planificacion + seguimiento + evasion + FSM)
            'navigator = nav_gridmap.navigator_node:main',
            # Parte C - Percepcion de cono rojo y publicacion de goal navegable
            'red_cone_mission = nav_gridmap.red_cone_mission_node:main',
        ],
    },
)
