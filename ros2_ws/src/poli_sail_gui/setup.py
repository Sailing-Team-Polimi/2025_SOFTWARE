from setuptools import find_packages, setup

package_name = 'poli_sail_gui'

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
    maintainer='ros',
    maintainer_email='95103311+ScudeT@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_to_monitor = poli_sail_gui.send_to_monitor:main',
            'mock_publisher = poli_sail_gui.mock_publisher:main',
        ],
    },
)
