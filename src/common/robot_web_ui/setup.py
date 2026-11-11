import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_web_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'i18n_build.py', 'i18n_en.tsv']),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
        (os.path.join('share', package_name, 'template'), glob('template/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='robots.entech@gmail.com',
    description='Shared web UI for robots (theme, layout, camera streams) + an example robot node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'example_server = robot_web_ui.example_server:main',
        ],
    },
)
