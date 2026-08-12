from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tractor_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'paths'), glob('paths/*.csv')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nontanan',
    maintainer_email='nontanan@gensurv.com',
    description='MPC and GWO control for tractor-trailer system',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mpc_node = simulation.mpc_node:main',
            'gwo_tuner_node = simulation.gwo_tuner_node:main',
            'path_publisher_node = simulation.path_publisher:main',
            'vehicle_node = tractor_control.vehicle_node:main',
            'nmpc_control_node = tractor_control.nmpc_control_node:main',
            'mpc_dynamic_control_node = tractor_control.mpc_dynamic_control_node:main',
            'ackermann_teleop_node = tractor_control.ackermann_teleop_node:main',
            'tractor_control_node = tractor_control.tracter_control:main',
        ],
    },
)
