from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),

        (os.path.join('share/', package_name, 'description'), 
            glob('description/*/*.xacro')),

        (os.path.join('share/', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='95103311+ScudeT@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose=gui.pose:main',
            'base_link_bc=gui.base_link_bc_node:main',
            'boat_state_pub=gui.boat_state_pub_node:main',
            'quat2rpy=gui.quat_to_rpy:main'
        ],
    },
)
