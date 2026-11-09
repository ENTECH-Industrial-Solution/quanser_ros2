from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qcar2_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='robots.entech@gmail.com',
    description='Local web interface for QCar2: cameras, driving, mapping (lidar SLAM / V-SLAM) and Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_server = qcar2_web.web_server:main',
            'rgbd_bridge = qcar2_web.rgbd_bridge:main',
        ],
    },
)
