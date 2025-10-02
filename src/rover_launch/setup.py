from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Necessario per ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Installa il package.xml
        ('share/' + package_name, ['package.xml']),

        # ✅ Installa tutti i file .launch.py nella cartella launch/
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miriam',
    maintainer_email='m.vitolo47@studenti.unisa.it',
    description='Launch package for MIVIA Rover',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
