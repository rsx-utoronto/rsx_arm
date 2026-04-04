from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'arm_utilities'

setup(
    name=package_name,
    version='0.0.0',
    packages=['arm_utilities', 'arm_configs'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all config files from arm_configs/
        (os.path.join('share', package_name, 'arm_configs'),
            glob('arm_configs/*.yaml')),  # adjust extension if needed
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Arm utilities package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)