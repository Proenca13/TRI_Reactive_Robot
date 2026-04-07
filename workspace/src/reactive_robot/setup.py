from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'reactive_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.obj'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.STL'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joao-proenca',
    maintainer_email='henriqueproenca2004@hotmail.com',
    description='Reactive robot assignment package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'decision_node = reactive_robot.decision_node:main',
            'radar_node = reactive_robot.radar_node:main',
            'movement_node = reactive_robot.movement_node:main',
        ],
    },
)